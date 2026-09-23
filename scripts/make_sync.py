#!/usr/bin/env python3
"""算出「影片第 0 幀」對應到 rosbag 的哪個時間點，寫進每一趟的 run.json。

為什麼需要：影片是用**模擬時間**算的（frame_times.csv 給幀號↔模擬時間），
而 `ros2 bag play` 的 `--start-offset` 是以 **bag 的牆鐘起點**為基準。
兩邊差一個常數，不算出來就只能靠眼睛對，36 段會對到瘋掉。

作法：讀 bag 裡的 /clock，找出「模擬時間 = 影片第一幀的模擬時間」那一刻
對應的 bag 牆鐘時間，兩者相減就是 --start-offset 要填的秒數。

用法：  source setup_sim_env.sh && python3 scripts/make_sync.py recordings
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from run_layout import run_dirs


def first_frame_sim_time(run_dir: Path) -> float | None:
    f = run_dir / "frame_times.csv"
    if not f.exists():
        return None
    for ln in f.read_text().splitlines():
        if ln.startswith("frame"):
            continue
        parts = ln.split(",")
        if len(parts) == 2:
            return float(parts[1])
    return None


def clock_track(bag_dir: Path):
    """回傳 [(bag 牆鐘秒, 模擬時間秒), ...]，取自 /clock。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock

    # bag 是 file 級 zstd 壓縮，一般的 SequentialReader 會報
    # "invalid magic bytes in Header"，要用會解壓的那個 reader。
    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/clock"]))
    out = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic != "/clock":
            continue
        msg = deserialize_message(data, Clock)
        out.append((t_ns * 1e-9, msg.clock.sec + msg.clock.nanosec * 1e-9))
    return out


def main(root: Path) -> int:
    n_ok = 0
    for d in run_dirs(root):
        meta_path = d / "run.json"
        bags = list((d / "bag").glob("*"))
        bag_dir = next((b for b in bags if b.is_dir()), None)
        if not meta_path.exists() or bag_dir is None:
            continue
        t_video = first_frame_sim_time(d)
        track = clock_track(bag_dir)
        if t_video is None or not track:
            print(f"  {d.name}: 缺 frame_times.csv 或 /clock，跳過")
            continue
        wall0 = track[0][0]
        # 找最接近影片起始模擬時間的那一筆
        best = min(track, key=lambda kv: abs(kv[1] - t_video))
        offset = best[0] - wall0
        meta = json.loads(meta_path.read_text())
        meta["sync"] = {
            "video_first_frame_sim_s": round(t_video, 3),
            "bag_sim_at_start_s": round(track[0][1], 3),
            "bag_sim_at_end_s": round(track[-1][1], 3),
            "bag_play_start_offset_s": round(offset, 3),
            "howto": ("影片第 0 幀 = `ros2 bag play <bag> --clock "
                      f"--start-offset {offset:.2f}`。影片幀率 = 模擬時間 30 fps，"
                      "所以影片第 N 幀 = 該 offset 再加 N/30 秒。"),
        }
        meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2))
        print(f"  {d.name}: 影片起點模擬時間 {t_video:7.2f} s → "
              f"bag --start-offset {offset:6.2f} s "
              f"（bag 模擬時間 {track[0][1]:.1f}~{track[-1][1]:.1f} s）")
        n_ok += 1
    print(f"已寫入 {n_ok} 趟的 sync 資訊")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(Path(sys.argv[1] if len(sys.argv) > 1 else "recordings")))
