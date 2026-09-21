"""把移動障礙物的目標位姿寫進 USD stage（執行期）。

分成兩層：

- :func:`world_pose_at` 是純計算（map frame 運動 → world frame 位姿），
  不碰 USD，所以能在沒有 Isaac 的機器上逐項驗。
- :class:`MovingObstacleDriver` 只負責把算好的位姿寫進 stage。

⚠ 這些 prim 在 USD 端是 **kinematic rigid body**（見 build_ros_graph
.place_moving_obstacles）。kinematic 的語意是「位姿由外部指定、不受力」——
所以每個物理步覆寫 transform 是正確用法，PhysX 會據此更新碰撞體位置，
PhysX 光達的 raycast 才打得到移動中的它。
"""

from __future__ import annotations

import math

import ros_graph_spec as S
from obstacle_motion import position_at


def world_pose_at(obstacle, sim_time: float, floor_top: float):
    """回傳 ``(world_x, world_y, world_z, world_yaw)``。

    Args:
        obstacle:  一個 :class:`ros_graph_spec.MovingObstacle`。
        sim_time:  模擬時間 s（不是牆鐘）。
        floor_top: 走廊地板頂面的 world z，見 measure_corridor_floor_top。
    """
    mx, my, myaw = position_at(
        obstacle.waypoints, obstacle.speed,
        sim_time + obstacle.phase_s, obstacle.mode,
    )
    wx, wy, wyaw = S.map_to_world(mx, my, myaw)
    return (wx, wy, floor_top + obstacle.height / 2.0, wyaw)


class MovingObstacleDriver:
    """每個物理步把行人挪到該在的位置。"""

    def __init__(self, stage, moving_obstacles, floor_top: float, root: str):
        from pxr import UsdGeom

        self._obstacles = []
        self._floor_top = floor_top
        for m in moving_obstacles:
            prim = stage.GetPrimAtPath(f"{root}/{m.name}")
            if not prim or not prim.IsValid():
                continue
            xf = UsdGeom.Xformable(prim)
            ops = [o for o in xf.GetOrderedXformOps()
                   if o.GetOpType() == UsdGeom.XformOp.TypeTransform]
            if not ops:
                continue
            self._obstacles.append((m, ops[0]))

    def __len__(self) -> int:
        return len(self._obstacles)

    def update(self, sim_time: float) -> None:
        from pxr import Gf

        for m, op in self._obstacles:
            wx, wy, wz, wyaw = world_pose_at(m, sim_time, self._floor_top)
            scale = (Gf.Vec3d(1.0, 1.0, 1.0) if m.kind == "person"
                     else Gf.Vec3d(m.size_x, m.size_y, m.height))
            mat = Gf.Matrix4d(1.0)
            mat.SetScale(scale)
            mat = mat * Gf.Matrix4d(1.0).SetRotate(
                Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(wyaw)))
            mat = mat * Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(wx, wy, wz))
            op.Set(mat)

