#!/usr/bin/env python3
"""模擬 ROS 介面的驗收工具。對著跑起來的 Isaac Sim 執行。

驗的都是「實際量到的東西」，不是設定檔寫了什麼：
  - /clock 是否在發、模擬時間的實時倍率 (RTF)
  - 點雲的 frame_id、點數、發佈頻率
  - **點雲座標系**：用地板點的 z 判斷是感測器座標系還是車體/世界座標系
  - /tf 上有哪些 frame 對（方案 A 下 Isaac 不該發任何 TF）
  - /odom_gt 的 frame 與內容

用法：
    source sim_ws/setup_sim_env.sh
    python3 sim_ws/scripts/verify_sim_ros.py --seconds 20
"""

from __future__ import annotations

import argparse
import math
import time
from collections import defaultdict

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_msgs.msg import TFMessage

SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)

#: 實車 URDF：base_footprint→base_link 0.13 + base_link→velodyne_link 1.3
SENSOR_HEIGHT_M = 1.43


class Verifier(Node):
    def __init__(self, cloud_topic: str) -> None:
        super().__init__("verify_sim_ros")
        self.clock_t: list[float] = []
        self.cloud_t: list[float] = []
        self.clouds: list[tuple[str, int, np.ndarray]] = []
        self.tf_pairs: dict[tuple[str, str], int] = defaultdict(int)
        self.odom: Odometry | None = None
        self.wall0 = time.time()
        self.sim0: float | None = None
        self.sim_last: float | None = None

        self.create_subscription(Clock, "/clock", self._cb_clock, 10)
        self.create_subscription(PointCloud2, cloud_topic, self._cb_cloud, SENSOR_QOS)
        self.create_subscription(TFMessage, "/tf", self._cb_tf, 100)
        self.create_subscription(Odometry, "/odom_gt", self._cb_odom, 10)

    def _cb_clock(self, m: Clock) -> None:
        t = m.clock.sec + m.clock.nanosec * 1e-9
        if self.sim0 is None:
            self.sim0 = t
        self.sim_last = t
        self.clock_t.append(time.time())

    def _cb_cloud(self, m: PointCloud2) -> None:
        self.cloud_t.append(time.time())
        if len(self.clouds) < 3:
            pts = pc2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True)
            self.clouds.append((m.header.frame_id, m.width, np.asarray(pts, dtype=np.float64)))

    def _cb_tf(self, m: TFMessage) -> None:
        for tr in m.transforms:
            self.tf_pairs[(tr.header.frame_id, tr.child_frame_id)] += 1

    def _cb_odom(self, m: Odometry) -> None:
        self.odom = m


def _rate(stamps: list[float]) -> float:
    return (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--cloud-topic", default="/velodyne_points_ideal")
    args = ap.parse_args()

    rclpy.init()
    node = Verifier(args.cloud_topic)
    t_end = time.time() + args.seconds
    while time.time() < t_end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    wall = time.time() - node.wall0

    ok = True
    print("=" * 66)

    # --- /clock 與實時倍率 ---
    if node.sim0 is None:
        print("✘ /clock  完全沒收到 —— use_sim_time 的整條鏈都會壞")
        ok = False
    else:
        sim_elapsed = node.sim_last - node.sim0
        rtf = sim_elapsed / wall if wall > 0 else 0.0
        print(f"✔ /clock  {_rate(node.clock_t):6.1f} Hz   模擬推進 {sim_elapsed:.2f} s / 實際 {wall:.2f} s")
        print(f"          實時倍率 RTF = {rtf:.3f}" + ("  ⚠ 遠慢於實時" if rtf < 0.3 else ""))

    # --- 點雲 ---
    if not node.clouds:
        print(f"✘ {args.cloud_topic}  沒收到")
        ok = False
    else:
        frame, width, pts = node.clouds[-1]
        hz = _rate(node.cloud_t)
        mark = "✔" if frame == "velodyne_link" else "✘"
        print(f"{mark} {args.cloud_topic}  frame_id='{frame}'  {width} 點  {hz:.1f} Hz")
        if frame != "velodyne_link":
            ok = False

        # 座標系判定：找地板。感測器座標系下地板應在 z ≈ -1.43
        z = pts[:, 2]
        h, edges = np.histogram(z, bins=120, range=(-3.0, 3.0))
        floor_z = edges[int(np.argmax(h))] + (edges[1] - edges[0]) / 2
        print(f"          點雲 z 範圍 [{z.min():+.3f}, {z.max():+.3f}]，最密集 z = {floor_z:+.3f}")
        if abs(floor_z + SENSOR_HEIGHT_M) < 0.25:
            print(f"          → 地板在 z≈{floor_z:+.2f}，符合感測器高 {SENSOR_HEIGHT_M} m"
                  f" → **點雲是感測器座標系** ✔")
        elif abs(floor_z) < 0.25:
            print("          → 地板在 z≈0 → 點雲是世界/地面座標系，**不是感測器座標系** ✘")
            ok = False
        else:
            print(f"          → 地板 z={floor_z:+.2f} 不符任一預期，需人工判讀 ⚠")

    # --- TF（方案 A：Isaac 不該發任何 TF）---
    if node.tf_pairs:
        print(f"✘ /tf  Isaac 仍在發 {len(node.tf_pairs)} 組 TF（方案 A 應為 0 組）：")
        for (p, c), n in sorted(node.tf_pairs.items(), key=lambda x: -x[1]):
            print(f"        {p} → {c}   ({n} 次)")
        ok = False
    else:
        print("✔ /tf  Isaac 未發任何 TF（方案 A 正確）")

    # --- odom ---
    if node.odom is None:
        print("✘ /odom_gt  沒收到")
        ok = False
    else:
        o = node.odom
        p = o.pose.pose.position
        mark = "✔" if o.child_frame_id == "base_footprint" else "✘"
        print(f"{mark} /odom_gt  {o.header.frame_id} → {o.child_frame_id}   "
              f"pos=({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})")
        if o.child_frame_id != "base_footprint":
            ok = False

    print("=" * 66)
    print("結果：" + ("全部通過" if ok else "有項目未通過（見上方 ✘）"))
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
