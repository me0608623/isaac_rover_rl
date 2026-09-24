"""車體位姿的逐幀記錄與回放插值（純資料，不依賴 Isaac / ROS）。

為什麼需要它：論文影片要用 path tracing 算（RTX Real-Time 在這個場景全黑），
而 path tracing 會把 RTF 壓到 0.35，導航就跑壞了 ——
2026-09-22 實測：邊錄邊導航時 cmd_vel 被釘在 0.060 m/s（正常是 0.475），
200 模擬秒只走了 14 m，沒到終點。

所以拆成兩趟：
  第一趟  正常速度跑導航（不算圖），把車的世界位姿逐幀寫成 CSV
  第二趟  不跑物理、不跑 ROS，照 CSV 把車擺回去，用 path tracing 慢慢算圖
行人是由模擬時間決定的程序化步態，兩趟完全一致，不必額外記錄。
"""

from __future__ import annotations

from dataclasses import dataclass

HEADER = "t,x,y,z,qw,qx,qy,qz"


@dataclass(frozen=True)
class PoseSample:
    """某個模擬時刻的世界位姿。四元數是 (w, x, y, z)。"""

    t: float
    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]


def format_row(s: PoseSample) -> str:
    return (f"{s.t:.6f},{s.pos[0]:.6f},{s.pos[1]:.6f},{s.pos[2]:.6f},"
            f"{s.quat[0]:.8f},{s.quat[1]:.8f},{s.quat[2]:.8f},{s.quat[3]:.8f}")


def parse_rows(lines) -> list[PoseSample]:
    """壞掉的行直接跳過 —— 第一趟可能被 Ctrl-C 砍在寫到一半。"""
    out: list[PoseSample] = []
    for ln in lines:
        ln = ln.strip()
        if not ln or ln.startswith("t,"):
            continue
        parts = ln.split(",")
        if len(parts) != 8:
            continue
        try:
            v = [float(x) for x in parts]
        except ValueError:
            continue
        out.append(PoseSample(v[0], (v[1], v[2], v[3]), (v[4], v[5], v[6], v[7])))
    return out


def _slerp(qa, qb, u: float):
    """四元數球面插值，**走短弧**。

    ⚠ 相反號的四元數代表同一個姿態。不先對齊號就插值，車會在原地甩一大圈。
    """
    import math

    dot = sum(a * b for a, b in zip(qa, qb))
    if dot < 0.0:
        qb = tuple(-b for b in qb)
        dot = -dot
    if dot > 0.9995:                       # 幾乎同向，線性插值即可
        q = tuple(a + (b - a) * u for a, b in zip(qa, qb))
    else:
        th0 = math.acos(max(-1.0, min(1.0, dot)))
        th = th0 * u
        s0 = math.sin(th0)
        wa = math.sin(th0 - th) / s0
        wb = math.sin(th) / s0
        q = tuple(a * wa + b * wb for a, b in zip(qa, qb))
    n = math.sqrt(sum(x * x for x in q)) or 1.0
    return tuple(x / n for x in q)


def pose_at(samples, t: float) -> PoseSample:
    """取 ``t`` 時刻的位姿。範圍外**夾住**，不外插。

    外插會讓影片結尾的車沿最後一段速度飛出畫面。
    """
    if not samples:
        raise ValueError("位姿軌跡是空的 —— 第一趟沒有寫出任何取樣點")
    if t <= samples[0].t:
        return samples[0]
    if t >= samples[-1].t:
        return samples[-1]
    lo, hi = 0, len(samples) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if samples[mid].t <= t:
            lo = mid
        else:
            hi = mid
    a, b = samples[lo], samples[hi]
    span = b.t - a.t
    u = 0.0 if span <= 0 else (t - a.t) / span
    pos = tuple(pa + (pb - pa) * u for pa, pb in zip(a.pos, b.pos))
    return PoseSample(t, pos, _slerp(a.quat, b.quat, u))


def motion_window(samples, move_eps: float = 0.05,
                  lead: float = 1.0, tail: float = 2.0):
    """回傳「車真的在動」的模擬時間區間 ``(t0, t1)``，前後各留一點餘裕。

    為什麼需要：第一趟是先開 Isaac、再開 ROS 棧、再發初始位姿，車真正起步
    大概是模擬時間第 60~70 秒。整段照錄的話，每支影片開頭都有一分鐘的
    靜止畫面，而且那一分鐘的 path tracing 是白算的（約佔四成的算圖時間）。

    完全沒動時回傳整段 —— 回傳空區間會錄出 0 幀的影片，
    分不出是「車沒動」還是「流程壞了」。
    """
    if not samples:
        raise ValueError("位姿軌跡是空的")
    # ⚠ 判斷「有沒有在動」要看**相鄰樣本之間**的位移，不能看「離起點多遠」。
    #   看離起點多遠的話，車一旦開走，後面停下來的每一筆都還是算「動過」，
    #   結尾那段靜止畫面就裁不掉。
    k = max(1, len(samples) // 400)         # 約 0.2 s 的中央差分視窗
    moving_t = []
    for i in range(len(samples)):
        a = samples[max(0, i - k)]
        b = samples[min(len(samples) - 1, i + k)]
        d = max(abs(b.pos[j] - a.pos[j]) for j in range(2))
        if d > move_eps:
            moving_t.append(samples[i].t)
    if not moving_t:
        return (samples[0].t, samples[-1].t)
    t0 = max(samples[0].t, moving_t[0] - lead)
    t1 = min(samples[-1].t, moving_t[-1] + tail)
    return (t0, t1)


# ── 行人軌跡 ───────────────────────────────────────────────────────────
#
# 接上 ORCA 之後行人會**因應車的動作**閃避，所以他們的軌跡依賴車的軌跡。
# 第二遍回放時車是照第一遍的位姿播的，若行人重算一次 ORCA，
# 更新頻率與積分順序都與第一遍不同，軌跡會發散 —— 影片裡的人就跟
# rosbag 裡光達打到的人對不起來。所以行人也逐幀記下來，第二遍純播放。

CROWD_HEADER = "t,name,x,y,yaw,phase,speed"


@dataclass(frozen=True)
class CrowdSample:
    """某個模擬時刻、某個行人的狀態（map frame）。"""

    t: float
    name: str
    x: float
    y: float
    yaw: float
    phase: float
    speed: float


def format_crowd_row(s: CrowdSample) -> str:
    return (f"{s.t:.6f},{s.name},{s.x:.5f},{s.y:.5f},"
            f"{s.yaw:.6f},{s.phase:.6f},{s.speed:.4f}")


def parse_crowd_rows(lines) -> list[CrowdSample]:
    out: list[CrowdSample] = []
    for ln in lines:
        ln = ln.strip()
        if not ln or ln.startswith("t,"):
            continue
        p = ln.split(",")
        if len(p) != 7:
            continue
        try:
            out.append(CrowdSample(float(p[0]), p[1], float(p[2]), float(p[3]),
                                   float(p[4]), float(p[5]), float(p[6])))
        except ValueError:
            continue
    return out


def _lerp_phase(a: float, b: float, u: float) -> float:
    """相位內插，走短弧。

    ⚠ 相位是 [0, 2π) 會繞回 0。直接線性內插會在繞回那一刻倒退一整圈，
    畫面上腳步像被往回抽一下。
    """
    import math

    two_pi = 2.0 * math.pi
    d = (b - a + math.pi) % two_pi - math.pi
    return (a + d * u) % two_pi


def smooth_crowd_yaw(rows: list) -> list:
    """回放前把每個行人的朝向套上與第一遍相同的轉身限制（見 orca_crowd.turn_toward）。

    2026-09-24 之前錄的 crowd.csv 存的是**原始**朝向（快停下時一格翻 180°）。
    回放照抄會把抖動錄進影片；在這裡補做平滑，舊資料只要重錄回放即可修正。
    新資料第一遍已經平滑過，再套一次結果不變（已在限速內的序列不會被改）。
    """
    import dataclasses
    from collections import defaultdict

    from orca_crowd import FACING_MIN_SPEED_M_S, MAX_TURN_RATE_RAD_S, turn_toward

    by = defaultdict(list)
    for r in rows:
        by[r.name].append(r)
    out = []
    for name, rs in by.items():
        rs.sort(key=lambda r: r.t)
        prev_t, y = None, None
        for r in rs:
            dt = 0.0 if prev_t is None else max(0.0, r.t - prev_t)
            target = r.yaw if r.speed > FACING_MIN_SPEED_M_S else None
            y = turn_toward(y, target, MAX_TURN_RATE_RAD_S * dt) if y is not None else (
                r.yaw if target is None else target)
            prev_t = r.t
            out.append(dataclasses.replace(r, yaw=y))
    return out


def crowd_at(rows, t: float):
    """取 ``t`` 時刻每個行人的 ``(x, y, yaw, phase, speed)``。

    沒有紀錄時回空 dict（靜態情境本來就沒人在走），不丟例外。
    """
    import math
    from collections import defaultdict

    if not rows:
        return {}
    by_name = defaultdict(list)
    for r in rows:
        by_name[r.name].append(r)
    out = {}
    for name, rs in by_name.items():
        rs.sort(key=lambda r: r.t)
        if t <= rs[0].t:
            r = rs[0]
            out[name] = (r.x, r.y, r.yaw, r.phase, r.speed)
            continue
        if t >= rs[-1].t:
            r = rs[-1]
            out[name] = (r.x, r.y, r.yaw, r.phase, r.speed)
            continue
        lo, hi = 0, len(rs) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if rs[mid].t <= t:
                lo = mid
            else:
                hi = mid
        a, b = rs[lo], rs[hi]
        span = b.t - a.t
        u = 0.0 if span <= 0 else (t - a.t) / span
        # 朝向也走短弧
        dy = (b.yaw - a.yaw + math.pi) % (2 * math.pi) - math.pi
        out[name] = (a.x + (b.x - a.x) * u, a.y + (b.y - a.y) * u,
                     a.yaw + dy * u, _lerp_phase(a.phase, b.phase, u),
                     a.speed + (b.speed - a.speed) * u)
    return out
