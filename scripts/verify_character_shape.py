#!/usr/bin/env python3
"""驗證光達打到的是**人體形狀**，不是圓柱近似。

判別依據：水平寬度隨高度的變化。
  圓柱 → 每一層寬度都等於直徑（約 0.50 m），標準差趨近 0
  人體 → 肩約 0.45 m、頭約 0.18 m、腿分開，逐層寬度明顯起伏

只看「有沒有回波」無法分辨兩者，所以這支量的是輪廓。
"""

from __future__ import annotations

import argparse
import math

import numpy as np

# Character_10 相對感測器的位置（由出生點與角色 world 座標推得）
DEFAULT_RANGE_M = 7.06
DEFAULT_BEARING_DEG = 0.6


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/velodyne_points_ideal")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--range", type=float, default=DEFAULT_RANGE_M)
    ap.add_argument("--bearing", type=float, default=DEFAULT_BEARING_DEG)
    ap.add_argument("--halfwidth", type=float, default=0.6,
                    help="目標周圍的取樣半寬(m)，用來把人從牆裡切出來")
    ap.add_argument("--auto", action="store_true",
                    help="自動定位：角色會走動時用，沿視線找最近的非牆叢集")
    ap.add_argument("--wall-range", type=float, default=11.5,
                    help="超過此距離視為牆（--auto 用）")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    chunks: list[np.ndarray] = []

    class Rec(Node):
        def __init__(self):
            super().__init__("verify_character_shape")
            self.create_subscription(PointCloud2, args.topic, self.cb,
                                     qos_profile_sensor_data)

        def cb(self, msg):
            pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
            chunks.append(np.asarray(pts, dtype=np.float64))

    rclpy.init()
    node = Rec()
    t_end = node.get_clock().now().nanoseconds * 1e-9 + args.seconds
    while node.get_clock().now().nanoseconds * 1e-9 < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if not chunks:
        print("[FAIL] 沒收到點雲")
        return 1
    xyz = np.concatenate(chunks)
    print(f"收到 {len(chunks)} 幀，共 {len(xyz)} 點")

    if args.auto:
        # 角色在走動 → 沿視線掃描，找最近的「人尺度」叢集。
        # 牆壁距離固定且很遠，先排除；剩下最近的密集段就是人。
        br0 = math.radians(args.bearing)
        ang = np.arctan2(xyz[:, 1], xyz[:, 0])
        rng = np.hypot(xyz[:, 0], xyz[:, 1])
        beam = (np.abs(ang - br0) < math.radians(4.0)) & (rng < args.wall_range)
        cand = rng[beam]
        if len(cand) < 50:
            print("[FAIL] 視線方向沒有近距離回波")
            return 1
        hist, edges = np.histogram(cand, bins=np.arange(1.5, args.wall_range, 0.25))
        hot = np.where(hist > hist.max() * 0.25)[0]
        args.range = float(edges[hot[0]] + 0.125)      # 最近的密集段
        print(f"  自動定位：目標在 {args.range:.2f} m（掃描 {len(cand)} 點）")

    # 取目標附近的柱狀區域（sensor frame）
    br = math.radians(args.bearing)
    cx, cy = args.range * math.cos(br), args.range * math.sin(br)
    d = np.hypot(xyz[:, 0] - cx, xyz[:, 1] - cy)
    sel = xyz[d < args.halfwidth]
    if len(sel) < 30:
        print(f"[FAIL] 目標位置 (r={args.range:.2f} m, {args.bearing:+.1f}°) "
              f"附近只有 {len(sel)} 點 → 光達沒打到角色")
        return 1

    print(f"[OK] 目標附近取到 {len(sel)} 點  高度範圍 "
          f"{sel[:, 2].min():+.2f} ~ {sel[:, 2].max():+.2f} m（sensor frame）")

    # 逐層量橫向寬度
    print("\n  高度(對地)   點數   橫向寬度")
    widths = []
    SENSOR_H = 1.43
    for lo in np.arange(-1.3, 0.75, 0.15):
        layer = sel[(sel[:, 2] >= lo) & (sel[:, 2] < lo + 0.15)]
        if len(layer) < 3:
            continue
        # 橫向 = 垂直於視線的方向
        lat = -(layer[:, 0] - cx) * math.sin(br) + (layer[:, 1] - cy) * math.cos(br)
        w = lat.max() - lat.min()
        widths.append(w)
        bar = "█" * max(1, int(w * 40))
        print(f"  {lo + SENSOR_H + 0.075:5.2f} m   {len(layer):4d}   {w:5.2f} m  {bar}")

    if len(widths) < 3:
        print("\n[WARN] 有效分層太少，無法判斷輪廓")
        return 0
    wa = np.array(widths)
    cv = wa.std() / wa.mean() if wa.mean() > 0 else 0.0
    print(f"\n  逐層寬度：平均 {wa.mean():.2f} m  標準差 {wa.std():.2f} m  變異係數 {cv:.2f}")
    if cv < 0.10:
        print("  → 寬度幾乎不隨高度變化：這是**圓柱**輪廓")
    else:
        print("  → 寬度隨高度明顯起伏：這是**人體**輪廓（肩/頭/腰不同寬）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
