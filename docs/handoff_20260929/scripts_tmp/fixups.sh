#!/bin/bash
# 補跑：sa4r2 c27 dynamic 1~4（dynamic 停用角色 bug），sa4r2 c27 mixed run03 補錄回放（GPU Xid 109）
cd /home/aa/IsaacLab/sim_ws
say(){ echo "[$(date '+%F %T')] $*"; }
A="recordings/_作廢批次_只留紀錄/c27_dynamic_停用角色bug_20260923"
mkdir -p "$A"
for d in recordings/模型sa4r2/sa4r2_c27_dynamic_run0*; do mv "$d" "$A/"; done
say "補跑 dynamic 4 趟"
ONLY=sa4r2_c27_dynamic bash scripts/record_batch.sh
source ./setup_sim_env.sh >/dev/null 2>&1
TAG=sa4r2_c27_mixed_run03; DIR="recordings/模型sa4r2/$TAG"; FR="$DIR/frames"
say "補錄回放 $TAG"
rm -rf "$FR"
./replay.sh --pose-log "$DIR/pose.csv" --crowd-log "$DIR/crowd.csv" --scene "$DIR/scene.json" \
  --out "$FR" --scenario mixed --run-index 3 --fps 30 --width 1280 --height 720 > "$DIR/replay.log" 2>&1
grep -E "^\[replay\] (第一幀|完成|情境)" "$DIR/replay.log"
mkdir -p "$DIR/video"
for cam in topdown chase oblique; do
  n=$(ls "$FR/$cam" 2>/dev/null | wc -l)
  [ "$n" -lt 30 ] && { say "⚠ $cam 只有 $n 幀"; continue; }
  ffmpeg -y -nostdin -hide_banner -loglevel error -framerate 30 -i "$FR/$cam/rgb_%04d.png" \
    -c:v h264_nvenc -preset p5 -cq 23 -pix_fmt yuv420p "$DIR/video/${TAG}_${cam}.mp4" 2>>"$DIR/encode.log" \
    && say "$cam: $n 幀"
done
cp -f "$FR/frame_times.csv" "$DIR/frame_times.csv" 2>/dev/null; rm -rf "$FR"
say "收尾分析"
python3 scripts/make_sync.py recordings 2>&1 | grep -vE "^\[(WARN|INFO)\]" | tail -2
python3 scripts/localization_error.py recordings 2>&1 | grep -vE "^\[(WARN|INFO)\]" > reports/localization_error.txt; tail -3 reports/localization_error.txt
python3 scripts/check_recordings.py recordings > reports/recordings_health.txt 2>&1; tail -2 reports/recordings_health.txt
tmp=$(mktemp); python3 scripts/make_readme.py recordings > "$tmp" && [ -s "$tmp" ] && mv "$tmp" recordings/README.md && say "README $(wc -l < recordings/README.md) 行"
python3 scripts/make_browse_tree.py >/dev/null 2>&1 && say "中文索引已重建"
say "補跑全部完成"
