#!/usr/bin/env python3
"""從錄好的 rosbag 算**定位誤差**：車以為自己在哪 vs 實際在哪。

為什麼要專門寫一支：先前一路報的是「map→odom 漂移」，那個數字**本身就
包含刻意注入的里程計誤差**（odom_drift_injector 模擬輪子打滑，e_s≈1%/m），
所以它會動是設計使然，看它判斷不了定位好壞。同理傾角也不是健康指標 ——
2026-09-21 曾出現傾角 0.9°（看起來很健康）但位置實際錯 7.95 m。

真值來源刻意**不用** `/odom_gt`：它的 odom frame 原點是車的起點，不是
世界座標。改用第一遍寫的 `pose.csv`（直接從 Isaac 讀 base_link 的世界位姿），
再用 world→map 轉到 map frame，與 TF 的 map→base_footprint 比。

⚠ `/ndt_pose` 的內容是 **map→odom**，不是車姿（見 policy_params 的註解），
不可以拿它當估計位姿。

用法：
    source setup_sim_env.sh
    python3 scripts/localization_error.py recordings
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

#: pose.csv 記的是 base_link，TF 查的是 base_footprint。兩者水平只差 1.8 mm
#: （實測 base_link y=2.9452 / base_footprint y=2.9434），對公尺級的定位誤差
#: 可以忽略，但別在更精細的分析裡沿用這個假設。
BASE_LINK_VS_FOOTPRINT_M = 0.0018


def compose_2d(parent, child):
    """2D 變換串接：``parent ∘ child``，各為 ``(x, y, yaw)``。

    ⚠ 子變換要先轉到父座標系再相加。直接把 x/y 相加是常見錯法，
    車一轉彎誤差就爆掉。
    """
    px, py, pyaw = parent
    cx, cy, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return (px + cx * c - cy * s, py + cx * s + cy * c, pyaw + cyaw)


def yaw_error_deg(a: float, b: float) -> float:
    """兩個角度的最小夾角（度）。359° 與 1° 只差 2°。"""
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return abs(math.degrees(d))


def interp_series(series, t: float):
    """在 ``[(t, (x, y, yaw)), ...]`` 上取 ``t``；範圍外夾住。yaw 走短弧。"""
    if not series:
        raise ValueError("序列是空的 —— 沒有資料時不可以回 (0,0,0)，"
                         "那會讓誤差剛好等於車的位置，數字假得看不出來")
    if t <= series[0][0]:
        return series[0][1]
    if t >= series[-1][0]:
        return series[-1][1]
    lo, hi = 0, len(series) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    (ta, a), (tb, b) = series[lo], series[hi]
    u = 0.0 if tb == ta else (t - ta) / (tb - ta)
    dy = (b[2] - a[2] + math.pi) % (2.0 * math.pi) - math.pi
    return (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + dy * u)


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_tf_chain(bag_dir: Path):
    """回傳 ``{(parent, child): [(sim_t, (x, y, yaw)), ...]}``。

    時間用訊息 header 的 stamp（模擬時間），不是 bag 的牆鐘時間 ——
    兩者差一個常數，混用會讓誤差曲線整條位移。
    """
    import subprocess

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    # 錄製程序若在收尾前被砍掉，會留下沒有 metadata.yaml、也沒壓縮的 .mcap，
    # rosbag2 直接開會報 "Could not find metadata for bag"。reindex 可以重建。
    if not (bag_dir / "metadata.yaml").exists():
        print(f"    （{bag_dir.name} 缺 metadata，先 reindex）", flush=True)
        subprocess.run(["ros2", "bag", "reindex", str(bag_dir), "-s", "mcap"],
                       capture_output=True, timeout=600)

    # 壓縮過的要用 SequentialCompressionReader，沒壓縮的要用一般 reader；
    # 用錯會報 "invalid magic bytes in Header"。看 metadata 裡有沒有壓縮欄位。
    meta = (bag_dir / "metadata.yaml").read_text(errors="replace")
    compressed = "compression_format: zstd" in meta
    reader = (rosbag2_py.SequentialCompressionReader() if compressed
              else rosbag2_py.SequentialReader())
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf", "/tf_static"]))
    out: dict = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, TFMessage)
        for tr in msg.transforms:
            key = (tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/"))
            t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            out.setdefault(key, []).append(
                (t, (tr.transform.translation.x, tr.transform.translation.y,
                     _yaw(tr.transform.rotation))))
    for v in out.values():
        v.sort(key=lambda kv: kv[0])
    return out


def estimated_map_pose(chain, t: float):
    """map → base_footprint（估計位姿）。串 map→odom 與 odom→base_footprint。"""
    mo = chain.get(("map", "odom"))
    ob = chain.get(("odom", "base_footprint"))
    if not mo or not ob:
        return None
    return compose_2d(interp_series(mo, t), interp_series(ob, t))


def analyse(run_dir: Path):
    """回傳這一趟的定位誤差統計。"""
    import statistics as st

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import ros_graph_spec as S
    from pose_log import motion_window, parse_rows

    bag = next((b for b in (run_dir / "bag").glob("*") if b.is_dir()), None)
    pose_csv = run_dir / "pose.csv"
    if bag is None or not pose_csv.exists():
        return None
    truth = parse_rows(pose_csv.read_text().splitlines())
    if len(truth) < 2:
        return None
    t0, t1 = motion_window(truth)
    chain = read_tf_chain(bag)

    errs, yaws, times = [], [], []
    for s in truth:
        if not (t0 <= s.t <= t1):
            continue
        est = estimated_map_pose(chain, s.t)
        if est is None:
            continue
        gx, gy, _ = S.world_to_map(s.pos[0], s.pos[1], 0.0)
        errs.append(math.hypot(est[0] - gx, est[1] - gy))
        gyaw = 2.0 * math.atan2(s.quat[3], s.quat[0])
        yaws.append(yaw_error_deg(est[2], gyaw + S.WORLD_TO_MAP_YAW_RAD))
        times.append(s.t)
    if not errs:
        return None
    half = len(errs) // 2
    return {
        "n": len(errs),
        "median": st.median(errs),
        "p95": sorted(errs)[int(len(errs) * 0.95)],
        "max": max(errs),
        "first_half": st.mean(errs[:half]),
        "second_half": st.mean(errs[half:]),
        "yaw_median": st.median(yaws),
        "yaw_max": max(yaws),
    }


def main(root: Path) -> int:
    print(f"{'tag':26s}{'取樣':>6s}{'位置誤差 中位/p95/最大 (m)':>30s}"
          f"{'前半→後半':>14s}{'朝向誤差 中位/最大 (°)':>24s}")
    rows = []
    for d in sorted(p for p in root.iterdir() if p.is_dir()):
        if d.name.startswith("_"):
            continue
        r = analyse(d)
        if r is None:
            print(f"{d.name:26s}  （資料不全，跳過）")
            continue
        rows.append((d.name, r))
        print(f"{d.name:26s}{r['n']:6d}"
              f"{r['median']:12.3f}{r['p95']:8.3f}{r['max']:8.3f}"
              f"{r['first_half']:7.2f}→{r['second_half']:5.2f}"
              f"{r['yaw_median']:16.2f}{r['yaw_max']:8.2f}")
    if rows:
        import statistics as st
        print(f"\n  全部 {len(rows)} 趟：中位誤差 "
              f"{st.median([r['median'] for _, r in rows]):.3f} m，"
              f"最大 {max(r['max'] for _, r in rows):.3f} m")
        worse = sum(1 for _, r in rows if r['second_half'] > r['first_half'] * 1.3)
        print(f"  後半段比前半段惡化 30% 以上的趟數：{worse} / {len(rows)}"
              f"（漂移會累積的話這裡會偏高）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(Path(sys.argv[1] if len(sys.argv) > 1 else "recordings")))
