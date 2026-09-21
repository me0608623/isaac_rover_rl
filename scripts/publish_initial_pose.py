#!/usr/bin/env python3
"""啟動時餵一次 /initialpose，讓 NDT 能在 routing 節點處收斂。

為什麼需要：
    ndt.cpp:14 的 ``pre_trans_`` 初值是 Identity，也就是 NDT 假設車在 map 原點。
    c28 在 map frame 的位置是 (-5.67, -0.49)，距原點 5.7 m，遠超 NDT 的收斂半徑，
    開機必定不收斂。

    ndt.cpp:28 有訂閱 ``/initialpose``（PoseWithCovarianceStamped），
    ndt.cpp:129 把它解讀為 **map→base_link**，再用 odom→base_link 換算成 map→odom。
    所以只要發一次車在 map 的位姿即可。

    這樣可以保留 odom_drift_injector 的 ``zero_start:=true``（較保真，
    保住「開機定位」這一關），又不會因為初始猜測差太遠而不收斂。

用法：
    python3 publish_initial_pose.py --ros-args -p node_name:=c28 -p use_sim_time:=true
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_graph_spec as S  # noqa: E402

#: 初始猜測的不確定度。NDT 不吃 covariance，但 RViz 會用它畫橢圓。
DEFAULT_COV_XY = 0.25 ** 2
DEFAULT_COV_YAW = math.radians(10.0) ** 2


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("sim_initial_pose")
        self.declare_parameter("node_name", S.SPAWN_ROUTING_NODE)
        self.declare_parameter("csv_path", str(Path(__file__).resolve().parents[1] / S.ROUTING_STATION_JSON))
        self.declare_parameter("topic", "/initialpose")
        self.declare_parameter("map_frame", S.FRAMES.map)
        self.declare_parameter("repeat", 5)
        self.declare_parameter("period_s", 1.0)

        gp = self.get_parameter
        name = gp("node_name").value
        csv = Path(gp("csv_path").value)
        nodes = S.read_station_nodes(csv)
        # JSON 的 rooms 沒有朝向；用「面向走廊下一站」算 yaw，與生成時一致，
        # 否則 NDT 的初始猜測角度會差一大截。
        if name not in nodes:
            raise SystemExit(f"[sim_initial_pose] {csv} 沒有節點 {name}")
        self.x, self.y, _ = nodes[name]
        face = S.SPAWN_FACING_NODE
        if face in nodes and face != name:
            fx, fy, _ = nodes[face]
            self.yaw = math.atan2(fy - self.y, fx - self.x)
        else:
            self.yaw = 0.0
        self.map_frame = gp("map_frame").value
        self.remaining = int(gp("repeat").value)

        # TRANSIENT_LOCAL：晚啟動的 ndt_localizer 也收得到。
        # ndt.cpp 用 rclcpp::QoS(100)（VOLATILE），publisher 提供更強的耐久性仍相容。
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, gp("topic").value, qos)
        self.timer = self.create_timer(float(gp("period_s").value), self._tick)
        self.get_logger().info(
            f"initialpose <- routing '{name}': map ({self.x:.3f}, {self.y:.3f}) "
            f"yaw {math.degrees(self.yaw):.2f} deg"
        )

    def _build(self) -> PoseWithCovarianceStamped:
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = self.x
        m.pose.pose.position.y = self.y
        m.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = DEFAULT_COV_XY
        cov[35] = DEFAULT_COV_YAW
        m.pose.covariance = cov
        return m

    def _tick(self) -> None:
        self.pub.publish(self._build())
        self.remaining -= 1
        if self.remaining <= 0:
            self.get_logger().info("initialpose 已發送完畢")
            self.timer.cancel()


def main() -> None:
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
