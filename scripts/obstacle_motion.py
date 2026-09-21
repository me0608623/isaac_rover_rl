"""移動障礙物的運動模型與光達可見度計算（純函數，不依賴 Isaac / USD）。

抽成獨立模組的理由：這兩件事都是**可以算錯而且錯了很難察覺**的東西 ——
可見度算錯會得到「車撞得到但 policy 看不到」的障礙物（實際踩過，見
test_low_cart_is_completely_invisible_to_policy），運動算錯則會讓行人
速度與論文宣稱的數字對不上。純函數才能在沒有 GPU 的情況下逐項驗。
"""

from __future__ import annotations

import math

#: (x, y) 路徑點，map frame。
Waypoint = tuple[float, float]

#: 到達終點後的行為。
#:   "pingpong" — 原路折返（走廊來回最自然）
#:   "loop"     — 回到起點重來（適合環狀路徑）
#:   "once"     — 停在終點
MODES = ("pingpong", "loop", "once")


def lidar_visible_height(
    obstacle_height: float,
    sensor_height: float,
    z_filter: float,
    base_z: float = 0.0,
) -> float:
    """障礙物落在 policy 可見帶內的高度（m）；0 代表 policy 完全看不到。

    前處理的 ``z_filter`` 是在 **sensor frame** 濾 |z|，所以實際保留的是地板
    上方 ``[sensor_height - z_filter, sensor_height + z_filter]`` 這一層水平帶。
    低於帶下緣的東西（例如 0.9 m 的推車）在 72 維觀測裡完全不存在，
    但它的碰撞體還在 —— 車撞得到卻看不到。

    Args:
        obstacle_height: 障礙物自身高度。
        sensor_height:   光達離地高（本專案 0.13 + 1.30 = 1.43 m）。
        z_filter:        前處理的 z 濾波半寬。
        base_z:          障礙物底部離地高，預設貼地。
    """
    band_low = sensor_height - z_filter
    band_high = sensor_height + z_filter
    top = base_z + obstacle_height
    return max(0.0, min(top, band_high) - max(base_z, band_low))


def path_length(waypoints: list[Waypoint] | tuple[Waypoint, ...]) -> float:
    """折線總長（m）。單點或空路徑為 0。"""
    if len(waypoints) < 2:
        return 0.0
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(waypoints, waypoints[1:])
    )


def position_at(
    waypoints: list[Waypoint] | tuple[Waypoint, ...],
    speed: float,
    t: float,
    mode: str = "pingpong",
) -> tuple[float, float, float]:
    """沿折線等速前進，回傳時刻 t 的 ``(x, y, yaw)``。

    以「累積弧長反查線段」求值而非逐步累加，所以任意 t 可直接求解：
    不依賴上一步狀態，重啟模擬或跳時間都得到同一條軌跡，實驗才可重現。

    Args:
        waypoints: map frame 的 (x, y) 路徑點，至少一個。
        speed:     行進速率 m/s（成人步行約 1.2~1.4）。
        t:         模擬時間 s。
        mode:      終點行為，見 :data:`MODES`。
    """
    if not waypoints:
        raise ValueError("waypoints 不可為空")
    if mode not in MODES:
        raise ValueError(f"mode 必須是 {MODES} 之一，收到 {mode!r}")

    total = path_length(waypoints)
    if len(waypoints) == 1 or total == 0.0 or speed == 0.0:
        return (waypoints[0][0], waypoints[0][1], 0.0)

    travelled = speed * t
    reverse = False
    if mode == "loop":
        u = travelled % total
    elif mode == "pingpong":
        u = travelled % (2.0 * total)
        if u > total:                      # 折返段
            u = 2.0 * total - u
            reverse = True
    else:                                   # once
        u = min(max(travelled, 0.0), total)

    acc = 0.0
    for i, (a, b) in enumerate(zip(waypoints, waypoints[1:])):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        is_last = i == len(waypoints) - 2
        if seg > 0.0 and (acc + seg >= u or is_last):
            r = min(max((u - acc) / seg, 0.0), 1.0)
            dx, dy = b[0] - a[0], b[1] - a[1]
            if reverse:
                dx, dy = -dx, -dy
            # ⚠ -0.0 陷阱：atan2(-0.0, -2.0) = -π 而非 +π，折返時 yaw 會差 2π
            #   的符號。-0.0 == 0.0 為真，所以這行能把負零正規化掉。
            if dy == 0.0:
                dy = 0.0
            return (a[0] + (b[0] - a[0]) * r,
                    a[1] + (b[1] - a[1]) * r,
                    math.atan2(dy, dx))
        acc += seg

    last = waypoints[-1]
    return (last[0], last[1], 0.0)

