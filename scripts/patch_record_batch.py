#!/usr/bin/env python3
"""給 record_batch.sh 打兩個補丁（可重複執行，已套用就跳過）。

為什麼要用腳本而不是直接改：bash 是**邊讀邊執行**的，批次正在跑的時候改
檔案會讓那份執行中的實例行為錯亂。所以改動排在批次結束之後，由夜間排程
呼叫這支。

補丁一：CROWD_MODE 環境變數
    讓對照組可以跑 --crowd-mode path（固定路線行人）而不必改預設。

補丁二：rosbag 收尾等待
    原本 SIGINT 之後固定 sleep 5。200~300 MB 的 bag 做 file 級 zstd 壓縮
    可能要更久，被後續的 kill 砍掉就會留下沒有 metadata.yaml 的檔
    （rosbag2 直接開會報 "Could not find metadata for bag"）。
    改成等 metadata.yaml 出現，最多 90 秒。
"""

from __future__ import annotations

import sys
from pathlib import Path

CROWD_DECL = '''SPEED_RATE=${SPEED_RATE:-0.7}'''
CROWD_NEW = '''SPEED_RATE=${SPEED_RATE:-0.7}
#: 行人模式。orca(預設)=RVO2 互動避讓；path=固定折線等速往返（對照組用）。
CROWD_MODE=${CROWD_MODE:-orca}'''

RUN_OLD = '''    nohup ./run_sim.sh --scenario "$SCEN" --run-index "$IDX" \\
          --pose-log "$POSE" --crowd-log "$CROWD" \\
          > "$DIR/isaac_nav.log" 2>&1 &'''
RUN_NEW = '''    nohup ./run_sim.sh --scenario "$SCEN" --run-index "$IDX" \\
          --crowd-mode "$CROWD_MODE" \\
          --pose-log "$POSE" --crowd-log "$CROWD" \\
          > "$DIR/isaac_nav.log" 2>&1 &'''

BAG_OLD = '''    pkill -INT -f "ros2 ba[g] record" 2>/dev/null
    sleep 5'''
BAG_NEW = '''    pkill -INT -f "ros2 ba[g] record" 2>/dev/null
    # 等 bag 真的收尾（寫出 metadata.yaml）。固定 sleep 5 不夠：
    # 200~300 MB 的 bag 做 file 級 zstd 壓縮可能更久，被後續 kill 砍掉就會
    # 留下沒有 metadata 的檔，rosbag2 之後開不起來。
    for _ in $(seq 1 45); do
        [ -f "$BAG/$TAG/metadata.yaml" ] && break
        pgrep -f "ros2 ba[g] record" >/dev/null || break
        sleep 2
    done
    [ -f "$BAG/$TAG/metadata.yaml" ] \\
        || say "  ⚠ bag 沒收尾（缺 metadata.yaml），事後需 ros2 bag reindex"'''

META_OLD = '''    "crowd_mode": "orca",'''
META_NEW = '''    "crowd_mode": __import__("os").environ.get("CROWD_MODE", "orca"),'''


def main(path: Path) -> int:
    s = path.read_text()
    applied, skipped = [], []
    for name, old, new in (("CROWD_MODE 宣告", CROWD_DECL, CROWD_NEW),
                           ("run_sim 傳 --crowd-mode", RUN_OLD, RUN_NEW),
                           ("rosbag 收尾等待", BAG_OLD, BAG_NEW),
                           ("run.json 記 crowd_mode", META_OLD, META_NEW)):
        if new in s:
            skipped.append(name)
            continue
        if old not in s:
            print(f"  ✗ {name}：找不到定位點，中止（不要半套）", file=sys.stderr)
            return 2
        s = s.replace(old, new, 1)
        applied.append(name)
    path.write_text(s)
    for n in applied:
        print(f"  ✔ 已套用：{n}")
    for n in skipped:
        print(f"  – 已存在，跳過：{n}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(Path(sys.argv[1] if len(sys.argv) > 1
                               else "scripts/record_batch.sh")))
