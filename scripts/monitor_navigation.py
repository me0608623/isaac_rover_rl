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

import json

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
    ap.add_argument("--tag", default="", help="這次實驗的標籤，寫進 CSV 檔名")
    ap.add_argument("--sim-time", action="store_true",
                    help="逾時與耗時改用**模擬時間**（/clock）計。錄影時算圖會把"
                         "RTF 壓到 0.35 左右，用牆鐘計時會把 54 s 的一段當成 "
                         "154 s，跟先前驗證過的數字對不起來。")
    ap.add_argument("--log-dir", default="",
                    help="逐時刻記錄寫到這個目錄（每段一個 CSV）。彙總數字看不出"
                         "「在哪一段、因為什麼停下來」，要診斷就得有時間序列。")
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
    from std_msgs.msg import String
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from tf2_ros import Buffer, TransformListener
    import rclpy.time
    from campusrover_msgs.srv import RoutingPath

    state = {"traj": [], "minrng": [], "cmd": [], "t": [], "vo": [],
             "mo": [], "series": []}

    from rclpy.parameter import Parameter

    class Mon(Node):
        def __init__(self):
            overrides = ([Parameter("use_sim_time", Parameter.Type.BOOL, True)]
                         if args.sim_time else [])
            super().__init__("monitor_navigation",
                             parameter_overrides=overrides)
            self.tf_buf = Buffer()
            self.tf_lis = TransformListener(self.tf_buf, self)
            self.create_subscription(PointCloud2, "/velodyne_points",
                                     self.on_cloud, qos_profile_sensor_data)
            self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
            # VO 安全煞的介入狀態（""/slow/stop/reverse/freeze）。
            # 這比成功率更能說明「policy 有多少次撐不住、需要硬底線接手」。
            self.create_subscription(String, "/vo_safety_node/status",
                                     self.on_vo, 10)
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
            # ⚠ NDT 漂移的定義就是「map→odom 本該準靜態，卻隨時間變動」。
            #   走廊導航是平面問題，健康時 roll/pitch 應貼近 0；
            #   2026-09-21 曾飄到 z=-1.95 m、roll=-6.3°，導航隨之亂繞。
            try:
                mo = self.tf_buf.lookup_transform("map", "odom", rclpy.time.Time())
            except Exception:
                return
            q = mo.transform.rotation
            roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                              1 - 2 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
            state["mo"].append((mo.transform.translation.x,
                                mo.transform.translation.y,
                                mo.transform.translation.z, roll, pitch))
            # 逐時刻快照：位置 + 當下 VO 狀態 + 當下最近障礙 + 當下命令速度。
            # 四者同一時間戳才能回答「它在哪裡、為什麼停」。
            state["series"].append((
                state["t"][-1], t.x, t.y,
                state["vo"][-1] if state["vo"] else "",
                state["minrng"][-1] if state["minrng"] else float("nan"),
                state["cmd"][-1][0] if state["cmd"] else float("nan"),
                math.degrees(abs(roll)), math.degrees(abs(pitch)),
            ))

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

        def on_vo(self, m):
            try:
                d = json.loads(m.data)
            except Exception:
                return
            state["vo"].append(str(d.get("front_brake", "") or ""))

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
        vo = state["vo"]
        if vo:
            act = [v for v in vo if v]
            from collections import Counter
            tally = Counter(act)
            detail = " ".join(f"{k}×{v}" for k, v in tally.most_common())
            print(f"  VO 安全煞介入 {len(act)}/{len(vo)} 幀 "
                  f"({len(act)/len(vo)*100:.1f}%)" + (f"　{detail}" if detail else ""))
        else:
            print("  VO 安全煞：沒收到 status（節點沒跑？）")
        mo = np.array(state["mo"]) if state["mo"] else None
        if mo is not None and len(mo) > 5:
            dxy = np.hypot(mo[:, 0] - mo[0, 0], mo[:, 1] - mo[0, 1]).max()
            dz = np.abs(mo[:, 2] - mo[0, 2]).max()
            rp = np.degrees(np.abs(mo[:, 3:5])).max()
            print(f"  NDT map→odom 漂移：水平 {dxy:.3f} m  垂直 {dz:.3f} m  "
                  f"|roll|,|pitch| 最大 {rp:.2f}°"
                  + ("  ⚠ 傾斜異常" if rp > 3.0 else ""))
        if args.log_dir:
            import csv, os
            os.makedirs(args.log_dir, exist_ok=True)
            tag = f"{args.tag}_" if args.tag else ""
            fn = os.path.join(args.log_dir, f"{tag}leg{leg_i}_{origin}_to_{goal}.csv")
            with open(fn, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["t", "map_x", "map_y", "vo_state", "min_range_m",
                            "cmd_v", "roll_deg", "pitch_deg"])
                t_zero = state["series"][0][0] if state["series"] else 0.0
                for row in state["series"]:
                    w.writerow([f"{row[0]-t_zero:.2f}"] + [f"{v:.3f}" if isinstance(v, float)
                               else v for v in row[1:]])
            print(f"  逐時刻記錄 → {fn}（{len(state['series'])} 筆）")

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
