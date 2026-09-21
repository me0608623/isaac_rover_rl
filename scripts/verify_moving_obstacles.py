#!/usr/bin/env python3
"""驗證 PhysX 光達確實掃得到**移動中**的行人。

為什麼需要單獨驗：幾個月前在這個場景放過 omni.anim.people 角色，RViz 看得到
人，光達卻完全沒有回波（RTX 光達不對骨架網格 raytrace）。「看得到」與
「掃得到」是兩回事，所以驗收標準必須是點雲本身，不是畫面。

作法：車靜止時，牆壁產生的 72-bin sweep 逐格應該不隨時間變化。任何隨時間
**週期性起伏**的 bin 就是移動物體的回波。再比對起伏週期與行人的往返週期。

用法：
    source sim_ws/setup_sim_env.sh
    python3 sim_ws/scripts/verify_moving_obstacles.py --seconds 20
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np

NUM_BINS = 72
Z_FILTER = 0.5          # lidar_preprocessor_params_sa4r2.yaml
R_MAX = 20.0


def sweep_from_cloud(xyz: np.ndarray) -> np.ndarray:
    """點雲（sensor frame）→ 72-bin 最近距離。與前處理同樣先濾 z 再 min-pool。"""
    keep = np.abs(xyz[:, 2]) <= Z_FILTER
    pts = xyz[keep]
    out = np.full(NUM_BINS, R_MAX, dtype=np.float64)
    if pts.size == 0:
        return out
    rng = np.hypot(pts[:, 0], pts[:, 1])
    ang = np.arctan2(pts[:, 1], pts[:, 0])
    idx = ((ang + math.pi) / (2 * math.pi) * NUM_BINS).astype(int) % NUM_BINS
    np.minimum.at(out, idx, np.minimum(rng, R_MAX))
    return out


def dominant_period(series: np.ndarray, dt: float) -> float:
    """用自相關估主週期（s）；找不到回傳 0。"""
    x = series - series.mean()
    if np.allclose(x, 0):
        return 0.0
    ac = np.correlate(x, x, mode="full")[len(x) - 1:]
    ac /= ac[0] if ac[0] != 0 else 1.0
    # 跳過零延遲附近，找第一個明顯峰
    start = max(2, int(0.4 / dt))
    if start >= len(ac) - 1:
        return 0.0
    seg = ac[start:]
    peak = int(np.argmax(seg)) + start
    return peak * dt if seg.max() > 0.3 else 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--topic", default="/velodyne_points")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    frames: list[np.ndarray] = []
    stamps: list[float] = []

    class Rec(Node):
        def __init__(self):
            super().__init__("verify_moving_obstacles")
            self.create_subscription(PointCloud2, args.topic, self.cb,
                                     qos_profile_sensor_data)

        def cb(self, msg):
            pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
            frames.append(sweep_from_cloud(np.asarray(pts, dtype=np.float64)))
            stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    rclpy.init()
    node = Rec()
    t_end = node.get_clock().now().nanoseconds * 1e-9 + args.seconds
    while node.get_clock().now().nanoseconds * 1e-9 < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if len(frames) < 10:
        print(f"[FAIL] 只收到 {len(frames)} 幀點雲，無法判定（Isaac 有在跑嗎？）")
        return 1

    arr = np.stack(frames)                       # (T, 72)
    dt = (stamps[-1] - stamps[0]) / max(1, len(stamps) - 1)
    std = arr.std(axis=0)
    rng = arr.max(axis=0) - arr.min(axis=0)

    print(f"收到 {len(frames)} 幀，間隔 {dt*1000:.1f} ms（{1/dt:.1f} Hz）")
    print(f"逐 bin 距離標準差：最大 {std.max():.3f} m / 中位數 {np.median(std):.3f} m")

    moving = np.where(rng > 0.3)[0]              # 起伏 > 30 cm 視為有動態回波
    if moving.size == 0:
        print("[FAIL] 沒有任何 bin 隨時間變化 → 光達掃不到移動的東西")
        return 1

    print(f"[OK] {moving.size} 個 bin 有動態回波（起伏 > 0.30 m）")
    order = moving[np.argsort(-rng[moving])][:5]
    for b in order:
        deg = (b + 0.5) * 360.0 / NUM_BINS - 180.0
        per = dominant_period(arr[:, b], dt)
        print(f"  bin {b:2d} (方位 {deg:+6.1f}°)  距離 {arr[:, b].min():5.2f}~{arr[:, b].max():5.2f} m"
              f"  起伏 {rng[b]:4.2f} m  主週期 {per:4.1f} s" if per else
              f"  bin {b:2d} (方位 {deg:+6.1f}°)  距離 {arr[:, b].min():5.2f}~{arr[:, b].max():5.2f} m"
              f"  起伏 {rng[b]:4.2f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
