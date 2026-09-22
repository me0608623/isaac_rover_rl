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
