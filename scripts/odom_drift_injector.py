#!/usr/bin/env python3
"""odom 系統性漂移注入 — 把 Isaac Sim 的真值里程計轉成「實車 driver 會算出來的」odom.

為什麼需要這支：
  車端 NDT 的 source 點雲是用 `odom→sensor` TF 轉進 odom frame 的
  （ndt.cpp:468-488, processInputCloud），NDT 解的是 map→odom。
  也就是說 **NDT 的輸入品質 = odom 的品質**：odom 一抖，source 點雲就變形，
  NDT 解跟著跳。實車實測正是如此 —— 靜止時 map→odom 逐幀只變 5.6mm，
  車一動就平均跳 0.134m（24 倍）。

  因此 Isaac Sim 若直接把真值 odom 餵給 NDT，NDT 會穩到完全不像實車，
  等於白跑。這支節點負責把真值劣化成實車該有的樣子。

模型（確定性，Borenstein & Feng UMBmark 系統性誤差）：
  真實車體位移 (Δs, Δθ) → 換算成真實軸距 b_t 下的左右輪位移
      u_L = Δs - Δθ·b_t/2
      u_R = Δs + Δθ·b_t/2
  driver 用「標定值」讀回來，標定/真實輪徑比為 λ_L, λ_R
      u_L' = λ_L·u_L      u_R' = λ_R·u_R
  driver 再用「標定軸距」b_a 反推
      Δs' = (u_L' + u_R')/2
      Δθ' = (u_R' - u_L')/b_a

  三個知覺化旋鈕：
      e_s  里程尺度誤差   λ_mean = 1 + e_s        （odom 報的距離 / 真實距離 - 1）
      e_d  左右輪徑不對稱 λ_R/λ_L ≈ 1 + e_d       （直行曲率漂移 κ ≈ e_d / b_t）
      e_b  軸距標定誤差   b_a = b_t·(1 + e_b)     （純旋轉的角度誤差比例）

  全部確定性、無隨機源 → 同一條軌跡每次跑出完全相同的漂移，可做 A/B 對照。

實車真值（/home/aa/rover2_ws/.../config/driver_chgh.yaml）：
      wheel_base_length     0.559212 m
      left_wheel_diameter   0.244211 m
      right_wheel_diameter  0.239577 m   → 左右不對稱 1.92%
      publish_rate          20.0 Hz
      base_frame            base_footprint     odom_frame  odom

用法（PC 端，Isaac Sim ROS2 bridge 已在發真值 odom）：
  ros2 run rover_rl_inference odom_drift_injector          # 若已納入 package
  python3 scripts/odom_drift_injector.py --ros-args \
      -p input_topic:=/odom_gt -p output_topic:=/odom \
      -p e_s:=-0.010 -p e_d:=0.0049

驗證：
  ros2 topic echo /odom_drift_injector/status      # 累積漂移量 JSON
  ros2 service call /odom_drift_injector/reset std_srvs/srv/Trigger   # 歸零重跑
"""
from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

# 實車 driver 20Hz 發 odom，QoS 用 reliable 小深度（與 campusrover_base 一致）
ODOM_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


def yaw_from_quat(q) -> float:
    """四元數取 yaw（平面車輛只用 z 軸旋轉）。"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quat_from_yaw(yaw: float):
    """yaw → 四元數 (x, y, z, w)。"""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def wrap_pi(a: float) -> float:
    """角度歸一到 [-π, π)。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class OdomDriftInjector(Node):
    """訂閱真值 odom，輸出帶確定性系統性漂移的 odom + TF。"""

    def __init__(self) -> None:
        super().__init__("odom_drift_injector")

        # ── 介面 ──
        self.declare_parameter("input_topic", "/odom_gt")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("output_rate_hz", 20.0)   # 實車 driver publish_rate

        # ── 漂移模型 ──
        # e_s: 里程尺度誤差。預設 -0.010 = odom 少報 1%（對應車端文件「drift 1%/m」）
        self.declare_parameter("e_s", -0.010)
        # e_d: 左右輪徑不對稱 → 直行曲率 κ = e_d/b_t。
        #      預設 0.0049 → κ=0.00876 rad/m ≈ 0.5°/m 航向漂移
        self.declare_parameter("e_d", 0.0049)
        # e_b: 軸距標定誤差。預設 0 = 不引入純旋轉誤差（需要時再開）
        self.declare_parameter("e_b", 0.0)
        self.declare_parameter("wheel_base_true", 0.559212)   # driver_chgh.yaml

        # ── 起點慣例 ──
        # True  = 漂移 odom 從 (0,0,0) 起算，複刻實車「開機即 odom 原點」
        #         → map→odom 初值 = 車在地圖中的真實起始位姿，NDT 必須找得到它
        #           （deploy_full 給 NDT 的初始猜測是 0,0,0 → sim 車請生在地圖原點附近）
        # False = 漂移 odom 從真值起始位姿起算 → map→odom 初值≈單位陣，NDT 最好收斂
        self.declare_parameter("zero_start", True)

        # ── 斷流模擬（實車 /odom 會間歇斷流 0.6~2.9s，最長實測 58.195s）──
        # 扁平清單 [起點秒, 持續秒, 起點秒, 持續秒, ...]，時間基準為收到第一筆真值起算。
        # 空清單 = 關閉。確定性排程，不是隨機。
        self.declare_parameter("dropout_schedule", [])
        # 斷流結束時是否 burst 補送期間累積的訊息（實車實測會 burst）
        self.declare_parameter("burst_on_resume", True)

        g = self.get_parameter
        self.in_topic = g("input_topic").value
        self.out_topic = g("output_topic").value
        self.odom_frame = g("odom_frame").value
        self.base_frame = g("base_frame").value
        self.do_tf = bool(g("publish_tf").value)
        self.rate_hz = float(g("output_rate_hz").value)
        self.e_s = float(g("e_s").value)
        self.e_d = float(g("e_d").value)
        self.e_b = float(g("e_b").value)
        self.b_true = float(g("wheel_base_true").value)
        self.zero_start = bool(g("zero_start").value)
        self.burst = bool(g("burst_on_resume").value)

        sched = list(g("dropout_schedule").value or [])
        if len(sched) % 2 != 0:
            self.get_logger().warn("dropout_schedule 長度非偶數，捨棄最後一項")
            sched = sched[:-1]
        self.dropouts = [(float(sched[i]), float(sched[i + 1]))
                         for i in range(0, len(sched), 2)]

        # 由三個旋鈕導出輪徑比與標定軸距
        lam_mean = 1.0 + self.e_s
        self.lam_l = lam_mean * (1.0 - self.e_d * 0.5)
        self.lam_r = lam_mean * (1.0 + self.e_d * 0.5)
        self.b_assumed = self.b_true * (1.0 + self.e_b)

        # ── 狀態 ──
        self.prev_true = None        # (x, y, yaw) 上一筆真值
        self.acc = None              # [x, y, yaw] 累積的漂移 odom 位姿
        self.t0 = None               # 第一筆真值的 ROS 時間（秒）
        self.last_pub_t = None       # 上次輸出時間（秒），供降頻用
        self.pending = []            # 斷流期間累積、待 burst 的訊息
        self.n_in = 0
        self.n_out = 0
        self.n_dropped = 0
        self.true_dist = 0.0         # 真實累積里程
        self.odom_dist = 0.0         # odom 報的累積里程

        # ── ROS 介面 ──
        self.pub = self.create_publisher(Odometry, self.out_topic, ODOM_QOS)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.tf_bc = TransformBroadcaster(self) if self.do_tf else None
        self.sub = self.create_subscription(
            Odometry, self.in_topic, self.on_true_odom, ODOM_QOS)
        self.create_service(Trigger, "~/reset", self.on_reset)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f"odom 漂移注入啟動：{self.in_topic} → {self.out_topic}\n"
            f"  e_s={self.e_s:+.4f}（尺度）e_d={self.e_d:+.4f}（輪徑不對稱）"
            f" e_b={self.e_b:+.4f}（軸距）\n"
            f"  λ_L={self.lam_l:.6f} λ_R={self.lam_r:.6f} b_a={self.b_assumed:.6f}\n"
            f"  直行曲率 κ={self.e_d / self.b_true:.5f} rad/m "
            f"（{math.degrees(self.e_d / self.b_true):.3f}°/m）\n"
            f"  zero_start={self.zero_start} 斷流排程 {len(self.dropouts)} 段 "
            f"burst={self.burst}")

    # ── 核心 ──
    def on_true_odom(self, msg: Odometry) -> None:
        self.n_in += 1
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        if self.prev_true is None:
            self.prev_true = (x, y, yaw)
            self.acc = [0.0, 0.0, 0.0] if self.zero_start else [x, y, yaw]
            return

        # 1) 真值增量 → 車體座標的 (Δs, Δθ)
        px, py, pyaw = self.prev_true
        dx, dy = x - px, y - py
        dtheta = wrap_pi(yaw - pyaw)
        # 沿「上一拍車頭方向」投影（與差速輪里程積分慣例一致）
        ds = dx * math.cos(pyaw) + dy * math.sin(pyaw)
        self.prev_true = (x, y, yaw)
        self.true_dist += math.hypot(dx, dy)

        # 2) 套用系統性誤差
        ds_m, dth_m = self.corrupt(ds, dtheta)
        self.odom_dist += abs(ds_m)

        # 3) 積分進漂移 odom（中點積分，比前向歐拉準）
        mid = self.acc[2] + dth_m * 0.5
        self.acc[0] += ds_m * math.cos(mid)
        self.acc[1] += ds_m * math.sin(mid)
        self.acc[2] = wrap_pi(self.acc[2] + dth_m)

        # 4) 速度同樣劣化（policy obs[0:4] ego 直接吃 odom 速度）
        v_m, w_m = self.corrupt_twist(msg.twist.twist.linear.x,
                                      msg.twist.twist.angular.z)

        out = self.build_msg(msg, v_m, w_m)

        # 5) 斷流判定
        if self.in_dropout(t - self.t0):
            self.n_dropped += 1
            if self.burst:
                self.pending.append(out)
            return

        if self.pending:
            for m in self.pending:          # 恢復時 burst 補送（實車實測行為）
                self.emit(m)
            self.pending.clear()

        # 6) 降頻到實車 publish_rate
        # 用 90% 週期當閘門而非剛好一個週期：輸入本來就接近目標頻率時，
        # 時戳抖動會讓部分間隔略小於週期而被誤砍（實測 20Hz 進、20Hz 出只過 133/200）。
        # 留 10% 容差 → 同頻時全數通過，高頻輸入仍正確降頻。
        if self.rate_hz > 0.0 and self.last_pub_t is not None:
            if (t - self.last_pub_t) < (0.9 / self.rate_hz):
                return
        self.last_pub_t = t
        self.emit(out)

    def corrupt(self, ds: float, dtheta: float) -> tuple[float, float]:
        """真實 (Δs, Δθ) → driver 會算出的 (Δs', Δθ')。"""
        u_l = ds - dtheta * self.b_true * 0.5
        u_r = ds + dtheta * self.b_true * 0.5
        u_l *= self.lam_l
        u_r *= self.lam_r
        return (u_l + u_r) * 0.5, (u_r - u_l) / self.b_assumed

    def corrupt_twist(self, v: float, w: float) -> tuple[float, float]:
        """速度走同一條輪速鏈路（與位移一致，避免速度/位置互相矛盾）。"""
        return self.corrupt(v, w)

    def in_dropout(self, elapsed: float) -> bool:
        return any(s <= elapsed < s + d for s, d in self.dropouts)

    def build_msg(self, src: Odometry, v: float, w: float) -> Odometry:
        out = Odometry()
        out.header.stamp = src.header.stamp      # 沿用真值時戳（NDT 靠它查 TF）
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = self.acc[0]
        out.pose.pose.position.y = self.acc[1]
        out.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quat_from_yaw(self.acc[2])
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.twist.twist.linear.x = v
        out.twist.twist.angular.z = w
        out.pose.covariance = list(src.pose.covariance)
        out.twist.covariance = list(src.twist.covariance)
        return out

    def emit(self, msg: Odometry) -> None:
        self.pub.publish(msg)
        self.n_out += 1
        if self.tf_bc is None:
            return
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_bc.sendTransform(tf)

    # ── 觀察/控制 ──
    def on_reset(self, _req, resp):
        self.prev_true = None
        self.acc = None
        self.t0 = None
        self.last_pub_t = None
        self.pending.clear()
        self.true_dist = 0.0
        self.odom_dist = 0.0
        resp.success = True
        resp.message = "漂移累積已歸零"
        self.get_logger().info("漂移累積已歸零")
        return resp

    def publish_status(self) -> None:
        if self.acc is None:
            return
        scale_err = (self.odom_dist / self.true_dist - 1.0) if self.true_dist > 0.1 else 0.0
        self.status_pub.publish(String(data=json.dumps({
            "e_s": self.e_s, "e_d": self.e_d, "e_b": self.e_b,
            "kappa_rad_per_m": self.e_d / self.b_true,
            "odom_x": round(self.acc[0], 4),
            "odom_y": round(self.acc[1], 4),
            "odom_yaw_deg": round(math.degrees(self.acc[2]), 3),
            "true_dist_m": round(self.true_dist, 3),
            "odom_dist_m": round(self.odom_dist, 3),
            "scale_err_measured": round(scale_err, 5),
            "n_in": self.n_in, "n_out": self.n_out, "n_dropped": self.n_dropped,
            "in_dropout": self.in_dropout(0.0) if self.t0 is None else None,
        }, ensure_ascii=False)))


def main() -> None:
    rclpy.init()
    node = OdomDriftInjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass          # Ctrl+C / 外部關閉：安靜收工，不吐 traceback
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
