#!/usr/bin/env python3
"""把 Isaac 的理想點雲快照加上真實 VLP-16 的掃描運動拖影。

為什麼需要
----------
車端 2026-08-24 實測判定：**點雲缺逐點去畸變是 NDT 的主要誤差源**。
VLP-16 轉一圈約 100 ms，但驅動把整包點標成「掃描結束」時戳；
``ndt_localizer`` 的 ``processInputCloud`` 又只對整包做**一次**剛體變換
（ndt.cpp:468-488），於是 0.8 m/s × 100 ms = 8 cm 的畸變完全沒被補償。
實車量到移動時 map→odom 每次更新跳 0.134 m，靜止時只跳 5.6 mm，比值 24 倍。

Isaac 的 PhysX Lidar 輸出的是**單一瞬間的快照**（所有點同一時刻），
這個主要誤差源在模擬裡根本不存在 —— NDT 會穩到完全不像實車。

模型
----
靜止世界點 W 在 t 時刻被測得，感測器位姿 S(t)，記錄值 ``p_t = S(t)⁻¹·W``；
但整包被標成 t_end 的時戳，消費端誤以為世界位置是 ``S(t_end)·p_t``。
所以由理想快照 ``p_end = S(t_end)⁻¹·W`` 反推真實失真點：

    p_smear = Δ · p_end,   Δ = S(t)⁻¹·S(t_end)         （t → t_end 的相對運動）
            = R(ω·dt)·p_end + v·dt,   dt = t_end − t

dt 由方位角決定，這正是拖影的來源：

    dt = T_scan · (1 − ((φ − φ₀) mod 2π) / 2π)

φ₀ 是掃描起始方位（VLP-16 從 0° 起掃），φ = atan2(y, x)。

已知限制（論文須誠實陳述）
--------------------------
本節點是對**單一快照做幾何扭曲**，重現的是點位移（主效應，8 cm 量級）。
真實掃描中不同方位是從不同位置量測的，因此還有視差與遮蔽變化 —— 這部分
無法由單一快照還原，屬二階效應（牆面在 5–20 m 時可忽略）。
若要完全物理正確，須改用 RTX Lidar 的旋轉掃描模式，但代價是
RTX Lidar 不對骨架網格行人做光線追蹤（論文 §2.8.2 已載明），行人會照不到。

速度來源
--------
拖影由**真實**運動造成，所以吃 ``/odom_gt``（Isaac 真值）而非漂移後的 ``/odom``。

用法
----
    python3 lidar_motion_smear.py --ros-args \\
        -p input_topic:=/velodyne_points_ideal \\
        -p output_topic:=/velodyne_points \\
        -p odom_topic:=/odom_gt \\
        -p scan_period_s:=0.1 \\
        -p use_sim_time:=true
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from lidar_smear_model import smear_points

SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)


class LidarMotionSmear(Node):
    def __init__(self) -> None:
        super().__init__("lidar_motion_smear")
        self.declare_parameter("input_topic", "/velodyne_points_ideal")
        self.declare_parameter("output_topic", "/velodyne_points")
        self.declare_parameter("odom_topic", "/odom_gt")
        self.declare_parameter("scan_period_s", 0.1)      # VLP-16 rpm=600
        self.declare_parameter("azimuth_start_deg", 0.0)
        self.declare_parameter("enabled", True)

        gp = self.get_parameter
        self.scan_period = float(gp("scan_period_s").value)
        self.az0 = math.radians(float(gp("azimuth_start_deg").value))
        self.enabled = bool(gp("enabled").value)
        self.vx = self.vy = self.omega = 0.0
        self._n_msgs = 0
        self._max_disp = 0.0

        self.pub = self.create_publisher(PointCloud2, gp("output_topic").value, SENSOR_QOS)
        self.create_subscription(PointCloud2, gp("input_topic").value, self._cb_cloud, SENSOR_QOS)
        self.create_subscription(Odometry, gp("odom_topic").value, self._cb_odom, 10)
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"motion smear {'ON' if self.enabled else 'OFF'}  "
            f"T_scan={self.scan_period*1000:.0f} ms  "
            f"{gp('input_topic').value} -> {gp('output_topic').value}"
        )

    def _cb_odom(self, msg: Odometry) -> None:
        # twist 已在 child_frame（base_footprint）下；對 VLP-16 的平移影響可忽略
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.omega = msg.twist.twist.angular.z

    def _cb_cloud(self, msg: PointCloud2) -> None:
        if not self.enabled:
            self.pub.publish(msg)
            return
        pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=False)
        if len(pts) == 0:
            self.pub.publish(msg)
            return
        out = smear_points(pts.astype(np.float64), self.vx, self.vy, self.omega,
                           self.scan_period, self.az0)
        self._max_disp = max(self._max_disp,
                             float(np.abs(out - pts).max()) if len(out) else 0.0)
        self._n_msgs += 1
        # 保留原時戳與 frame_id —— NDT 靠它查 TF，絕不可改
        new = pc2.create_cloud_xyz32(msg.header, out.astype(np.float32))
        self.pub.publish(new)

    def _report(self) -> None:
        if self._n_msgs:
            self.get_logger().info(
                f"smear: {self._n_msgs} clouds, 本期最大位移 {self._max_disp*100:.1f} cm "
                f"(v={math.hypot(self.vx, self.vy):.2f} m/s, w={self.omega:.2f} rad/s)")
            self._n_msgs = 0
            self._max_disp = 0.0


def main() -> None:
    rclpy.init()
    node = LidarMotionSmear()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
