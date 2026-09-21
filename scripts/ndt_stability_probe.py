#!/usr/bin/env python3
"""量 NDT 在**車靜止**時的解算穩定度。

為什麼靜止量：車不動 → odom 不動 → map→odom 應該是常數。任何變動都是
純粹的掃描匹配不穩定，不會混進里程漂移的補償量（odom_drift_injector 正在
刻意注入誤差，移動中量會分不清是誰的問題）。

判準：走廊導航是平面問題，健康的 map→odom 其 roll/pitch 應貼近 0 且不隨
時間增長。2026-09-21 實測導航失敗時傾角爬到 18.76°，即為 NDT 鎖進錯誤的
傾斜解。

用法：
    python3 ndt_stability_probe.py --seconds 60 --tag chars_off
"""

from __future__ import annotations

import argparse
import math


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--tag", default="", help="這次條件的標籤")
    args = ap.parse_args()

    import numpy as np
    import rclpy
    import rclpy.time
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    samples: list[tuple[float, float, float, float, float, float]] = []

    class Probe(Node):
        def __init__(self):
            super().__init__("ndt_stability_probe")
            self.buf = Buffer()
            self.lis = TransformListener(self.buf, self)
            self.t0 = None
            self.create_timer(0.1, self.tick)

        def tick(self):
            try:
                tf = self.buf.lookup_transform("map", "odom", rclpy.time.Time())
            except Exception:
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.t0 is None:
                self.t0 = now
            t = tf.transform.translation
            q = tf.transform.rotation
            if math.isnan(t.x):
                return
            roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                              1 - 2 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
            samples.append((now - self.t0, t.x, t.y, t.z,
                            math.degrees(roll), math.degrees(pitch)))

    rclpy.init()
    node = Probe()
    t_end = node.get_clock().now().nanoseconds * 1e-9 + args.seconds
    while node.get_clock().now().nanoseconds * 1e-9 < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if len(samples) < 20:
        print(f"[FAIL] 只取到 {len(samples)} 筆 map→odom，NDT 有在發嗎？")
        return 1

    a = np.array(samples)
    t, x, y, z, roll, pitch = a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4], a[:, 5]
    tilt = np.abs(roll) + np.abs(pitch)
    dxy = np.hypot(x - x[0], y - y[0])

    print(f"[{args.tag or 'ndt'}] 取樣 {len(a)} 筆 / {t[-1]:.1f} s（車靜止）")
    print(f"  水平漂移  最大 {dxy.max():.3f} m   末值 {dxy[-1]:.3f} m")
    print(f"  垂直漂移  最大 {np.abs(z - z[0]).max():.3f} m")
    print(f"  |roll|+|pitch|  起始 {tilt[0]:.2f}°  最大 {tilt.max():.2f}°  末值 {tilt[-1]:.2f}°")
    # 傾角是否單調惡化（鎖進錯誤解的特徵）
    half = len(tilt) // 2
    growth = tilt[half:].mean() - tilt[:half].mean()
    print(f"  傾角後半段 − 前半段 = {growth:+.2f}°"
          + ("  ⚠ 持續惡化" if growth > 1.0 else "  （未持續惡化）"))
    verdict = ("穩定" if tilt.max() < 2.0 and dxy.max() < 0.1
               else "可疑" if tilt.max() < 5.0 else "不穩定")
    print(f"  → {verdict}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
