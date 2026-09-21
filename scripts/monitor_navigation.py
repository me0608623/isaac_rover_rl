#!/usr/bin/env python3
"""記錄一趟導航的關鍵指標，供事後判定成敗。

為什麼要獨立量：光看車有沒有到終點不足以判斷 policy 好壞 ——
可能是擦著障礙物過去的，也可能是停在原地被超時結束。這裡同時記錄
真值軌跡、最近障礙距離與速度，讓「抵達」「碰撞」「卡住」三者分得開。

真值用 /odom_gt（Isaac 直接給的世界座標），不是 /ndt_pose —— 後者是估計值，
用它評估會把定位誤差算進導航表現裡。
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np

#: 碰撞判定：LiDAR 量到的最近距離低於此值即視為撞上。
#: 0.45 = 車體半徑 0.35 + 緩衝 0.10（見專案 CLAUDE.md）。
COLLISION_RANGE_M = 0.45
#: 抵達判定半徑。
ARRIVE_RADIUS_M = 1.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--legs", default="c25",
                    help="依序要去的 routing 站名，逗號分隔。"
                         "例：c25,c28 = 去程 + 回程（來回對照）")
    ap.add_argument("--start", default="c28", help="第一段的起點站名")
    ap.add_argument("--seconds", type=float, default=90.0, help="每段的逾時")
    args = ap.parse_args()

    sys.path.insert(0, str(__file__.rsplit("/", 1)[0]))
    import ros_graph_spec as S

    stations = S.read_station_nodes(S.ROUTING_STATION_JSON)
    legs = [g.strip() for g in args.legs.split(",") if g.strip()]
    for g in legs:
        if g not in stations:
            print(f"[FAIL] 站名 {g} 不在 {S.ROUTING_STATION_JSON}")
            return 2

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from tf2_ros import Buffer, TransformListener
    import rclpy.time
    from campusrover_msgs.srv import RoutingPath

    state = {"traj": [], "minrng": [], "cmd": [], "t": []}

    class Mon(Node):
        def __init__(self):
            super().__init__("monitor_navigation")
            self.tf_buf = Buffer()
            self.tf_lis = TransformListener(self.tf_buf, self)
            self.create_subscription(PointCloud2, "/velodyne_points",
                                     self.on_cloud, qos_profile_sensor_data)
            self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
            self.create_timer(0.1, self.on_tick)
            self.routing = self.create_client(RoutingPath,
                                              "/routing_to_path/routing_call")

        def on_tick(self):
            try:
                tf = self.tf_buf.lookup_transform(
                    "map", "base_footprint", rclpy.time.Time())
            except Exception:
                return
            t = tf.transform.translation
            if math.isnan(t.x) or math.isnan(t.y):
                return          # 物理爆掉時會出現 NaN，別讓它污染統計
            state["traj"].append((t.x, t.y))
            state["t"].append(self.get_clock().now().nanoseconds * 1e-9)

        def on_cloud(self, m):
            pts = point_cloud2.read_points_numpy(m, field_names=("x", "y", "z"))
            a = np.asarray(pts, dtype=np.float64)
            if a.size == 0:
                return
            keep = np.abs(a[:, 2]) <= 0.5          # 同 policy 的 z_filter
            if not keep.any():
                return
            r = np.hypot(a[keep, 0], a[keep, 1])
            r = r[np.isfinite(r)]
            if r.size:
                state["minrng"].append(float(r.min()))

        def on_cmd(self, m):
            state["cmd"].append((m.linear.x, m.angular.z))

        def send_routing(self, origin: str, dest: str) -> bool:
            if not self.routing.wait_for_service(timeout_sec=10.0):
                return False
            req = RoutingPath.Request()
            req.origin, req.destination = origin, [dest]
            fut = self.routing.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
            return fut.done()

    rclpy.init()
    node = Mon()
    results = []
    origin = args.start

    for leg_i, goal in enumerate(legs, 1):
        gx, gy = stations[goal][0], stations[goal][1]
        print(f"\n── 第 {leg_i} 段：{origin} → {goal} @ map({gx:+.2f},{gy:+.2f}) ──")
        if not node.send_routing(origin, goal):
            print(f"  [FAIL] routing 呼叫失敗")
            break
        for k in state:
            state[k].clear()

        t0 = node.get_clock().now().nanoseconds * 1e-9
        arrived = None
        while node.get_clock().now().nanoseconds * 1e-9 - t0 < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
            if state["traj"]:
                mx, my = state["traj"][-1]
                if math.hypot(mx - gx, my - gy) < ARRIVE_RADIUS_M:
                    arrived = node.get_clock().now().nanoseconds * 1e-9 - t0
                    break

        tr = state["traj"]
        if len(tr) < 5:
            print("  [FAIL] 幾乎沒收到 TF map→base_footprint")
            break
        dist = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(tr, tr[1:]))
        gap = math.hypot(tr[-1][0] - gx, tr[-1][1] - gy)
        rng = np.array(state["minrng"]) if state["minrng"] else np.array([9.9])
        cmds = np.array(state["cmd"]) if state["cmd"] else np.zeros((1, 2))
        n_coll = int((rng < COLLISION_RANGE_M).sum())
        ok = arrived is not None

        print(f"  起點 map({tr[0][0]:+.2f},{tr[0][1]:+.2f})  終點 map({tr[-1][0]:+.2f},{tr[-1][1]:+.2f})")
        print(f"  {'抵達' if ok else '未達'}  耗時 {arrived if ok else args.seconds:.1f} s  "
              f"路徑 {dist:.2f} m  距目標 {gap:.2f} m")
        print(f"  最近障礙 最小 {rng.min():.2f} m / 中位 {np.median(rng):.2f} m　"
              f"碰撞幀 {n_coll}/{len(rng)}　|v| 平均 {np.abs(cmds[:,0]).mean():.3f}")
        results.append((f"{origin}→{goal}", ok, arrived if ok else args.seconds,
                        dist, gap, rng.min(), n_coll, len(rng)))
        origin = goal
        if not ok:
            print("  → 本段未達，停止後續段")
            break

    node.destroy_node()
    rclpy.shutdown()

    print("\n" + "=" * 62)
    print(f"{'段':12s} {'結果':6s} {'耗時':>7s} {'路徑':>7s} {'最近障礙':>8s} {'碰撞幀':>8s}")
    for name, ok, t, d, gap, mn, nc, tot in results:
        print(f"{name:12s} {'OK' if ok else 'FAIL':6s} {t:6.1f}s {d:6.2f}m "
              f"{mn:7.2f}m {nc:4d}/{tot:<4d}")
    print("=" * 62)
    return 0 if results and all(r[1] for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
