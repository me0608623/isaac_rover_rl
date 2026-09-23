"""用 ORCA（RVO2）驅動走廊行人，讓他們會**互相**與**對車**閃避。

為什麼要換掉原本的固定路線：原本行人沿折線等速來回，車來了也不會讓，
只能算「移動的障礙物」。使用者要的是會互動的行人。

ORCA（Optimal Reciprocal Collision Avoidance）是行人模擬的標準解，
`rvo2` 這顆原生函式庫已經裝在 Isaac 的 python 環境裡（env_isaaclab），
直接用正牌實作，不自己寫近似版。

怎麼接：
  * 每個行人是一個 ORCA agent，目標仍是原本的往返端點（走的路線沒變，
    只是遇到人或車會自己繞開、減速、等一下再走）。
  * **車也是一個 agent**，但每一步用模擬的真值覆寫它的位置與速度 ——
    車是被 policy 控制的，不受 ORCA 擺佈。車的半徑刻意放大，
    因為 ORCA 假設雙方各負一半避讓責任，而車根本不會讓。
  * 靜態障礙（推車、立柱）與走廊兩側牆用 RVO2 的 obstacle 表示。

⚠ 相位不能再用模擬時間算。ORCA 下行人會加減速甚至停住，
  腳步要改用「走過的距離」推進（gait.phase_advance），否則會滑步。

⚠ 這支讓行人的軌跡**依賴車的軌跡**，所以第二遍回放不能重算 ——
  必須照第一遍記下來的每個人的位姿播。見 pose_log 的多 agent 記錄。
"""

from __future__ import annotations

import math

#: 行人的碰撞半徑（m）。人體半徑約 0.25，留一點社交距離。
PEDESTRIAN_RADIUS_M = 0.30

#: 車在 ORCA 裡的半徑（m）。車體半徑 0.35，這裡刻意放大 ——
#: ORCA 假設雙方各負一半避讓責任，但車由 policy 控制、完全不讓，
#: 用原尺寸行人只會讓一半，照樣擦上。
ROBOT_AGENT_RADIUS_M = 0.75

#: 到目標多近算抵達、要換下一個端點（m）。
GOAL_TOLERANCE_M = 0.8

#: ORCA 參數。time_horizon 是「往前看幾秒避讓」。
NEIGHBOR_DIST_M = 5.0
MAX_NEIGHBORS = 10
TIME_HORIZON_S = 3.0
TIME_HORIZON_OBST_S = 1.0


def next_goal_index(idx: int, pos, waypoints, tol: float = GOAL_TOLERANCE_M) -> int:
    """到了就換另一端，否則維持原目標（往返 = 兩端點輪流）。"""
    gx, gy = waypoints[idx]
    if math.hypot(gx - pos[0], gy - pos[1]) <= tol:
        return (idx + 1) % len(waypoints)
    return idx


def pref_velocity(pos, goal, speed: float):
    """朝目標的偏好速度。站在目標上時回 (0, 0)。

    ⚠ 不要回正規化 NaN：NaN 餵進 ORCA 會讓整個 solver 的輸出變 NaN，
    行人會瞬移到天邊，而且不會報錯。
    """
    dx, dy = goal[0] - pos[0], goal[1] - pos[1]
    d = math.hypot(dx, dy)
    if d < 1e-6 or speed <= 0.0:
        return (0.0, 0.0)
    return (speed * dx / d, speed * dy / d)


def ccw_rect(cx: float, cy: float, half_x: float, half_y: float):
    """以 (cx, cy) 為中心的矩形，頂點**逆時針**排列。

    ⚠ RVO2 的靜態障礙要逆時針繞才代表「agent 待在外面」。
    繞反了 agent 會被推到障礙裡面（等於穿牆），而且不會有任何錯誤訊息。
    """
    return [(cx - half_x, cy - half_y), (cx + half_x, cy - half_y),
            (cx + half_x, cy + half_y), (cx - half_x, cy + half_y)]


def oriented_rect(cx: float, cy: float, half_x: float, half_y: float,
                  yaw: float):
    """旋轉 ``yaw``（rad）的矩形，頂點**逆時針**排列（RVO2 要求）。

    道具的長邊沿走廊擺，走廊又不是正東西向，所以不能用 ccw_rect。
    旋轉不改變繞行方向，所以 ccw_rect 的逆時針順序旋轉後仍是逆時針。
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in ((-half_x, -half_y), (half_x, -half_y),
                           (half_x, half_y), (-half_x, half_y))]


def polygon_area(verts) -> float:
    """有號面積：正 = 逆時針。拿來檢查給 RVO2 的障礙沒有繞反。"""
    n = len(verts)
    return sum(verts[i][0] * verts[(i + 1) % n][1] - verts[(i + 1) % n][0] * verts[i][1]
               for i in range(n)) / 2.0


class OrcaCrowd:
    """一群用 ORCA 走路的行人，外加一個由外部控制的車 agent。

    座標一律用 **map frame**（與 CharacterWalk 的 waypoints 同一個座標系）。
    """

    def __init__(self, walks, time_step: float,
                 obstacles=(), wall_bands=(),
                 robot_radius: float = ROBOT_AGENT_RADIUS_M):
        import rvo2

        self._sim = rvo2.PyRVOSimulator(
            time_step, NEIGHBOR_DIST_M, MAX_NEIGHBORS,
            TIME_HORIZON_S, TIME_HORIZON_OBST_S,
            PEDESTRIAN_RADIUS_M, 1.0)
        self._dt = time_step
        self._names: list[str] = []
        self._walks = []
        self._goal_idx: list[int] = []
        self._phase: list[float] = []
        self._ids: list[int] = []

        for w in walks:
            if not w.waypoints:
                continue
            aid = self._sim.addAgent(
                tuple(w.waypoints[0]), NEIGHBOR_DIST_M, MAX_NEIGHBORS,
                TIME_HORIZON_S, TIME_HORIZON_OBST_S,
                PEDESTRIAN_RADIUS_M, w.speed, (0.0, 0.0))
            self._ids.append(aid)
            self._names.append(w.name)
            self._walks.append(w)
            self._goal_idx.append(len(w.waypoints) - 1)
            self._phase.append(0.0)

        # 車：也是 agent，但每步由外部覆寫位置與速度。
        self._robot_id = self._sim.addAgent(
            (0.0, 0.0), NEIGHBOR_DIST_M, MAX_NEIGHBORS,
            TIME_HORIZON_S, TIME_HORIZON_OBST_S, robot_radius, 2.0, (0.0, 0.0))

        for verts in obstacles:
            self._sim.addObstacle(list(verts))
        for band in wall_bands:
            self._sim.addObstacle(list(band))
        self._sim.processObstacles()

    def __len__(self) -> int:
        return len(self._ids)

    @property
    def names(self):
        return tuple(self._names)

    def step(self, robot_xy, robot_vel):
        """推進一步，回傳 ``{name: (x, y, yaw, phase)}``（map frame）。"""
        self._sim.setAgentPosition(self._robot_id, tuple(robot_xy))
        self._sim.setAgentVelocity(self._robot_id, tuple(robot_vel))
        self._sim.setAgentPrefVelocity(self._robot_id, tuple(robot_vel))

        for k, aid in enumerate(self._ids):
            pos = self._sim.getAgentPosition(aid)
            w = self._walks[k]
            self._goal_idx[k] = next_goal_index(self._goal_idx[k], pos, w.waypoints)
            self._sim.setAgentPrefVelocity(
                aid, pref_velocity(pos, w.waypoints[self._goal_idx[k]], w.speed))

        before = [self._sim.getAgentPosition(a) for a in self._ids]
        self._sim.doStep()

        from gait import phase_advance

        out = {}
        for k, aid in enumerate(self._ids):
            pos = self._sim.getAgentPosition(aid)
            vel = self._sim.getAgentVelocity(aid)
            ds = math.hypot(pos[0] - before[k][0], pos[1] - before[k][1])
            spd = math.hypot(vel[0], vel[1])
            self._phase[k] = (self._phase[k]
                              + phase_advance(ds, max(spd, 1e-3))) % (2.0 * math.pi)
            # 朝向看行進方向；幾乎靜止時保留上一次的朝向（避免原地亂轉）。
            yaw = math.atan2(vel[1], vel[0]) if spd > 0.05 else None
            out[self._names[k]] = (pos[0], pos[1], yaw, self._phase[k], spd)
        return out
