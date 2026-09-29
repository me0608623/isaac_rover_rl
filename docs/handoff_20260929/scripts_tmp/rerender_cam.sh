#!/bin/bash
# 等 Character_04 重錄結束，再把「還是舊相機」的趟重做第二遍回放（斜前方/車後避牆，3a38cae）
cd /home/aa/IsaacLab/sim_ws
WS=$PWD
say(){ echo "[$(date '+%F %T')] $*"; }
L=$(ls -t reports/rerun_c04_*.log | head -1)
until grep -q "重錄全部完成" "$L"; do sleep 30; done
source ./setup_sim_env.sh >/dev/null 2>&1
DIRS=$(for f in recordings/模型*/*/run.json; do d=$(dirname "$f"); grep -q "\[camera\] 牆面格網：" "$d/replay.log" 2>/dev/null || echo "$d"; done)
N=$(echo "$DIRS" | grep -c .); say "要補錄回放 $N 趟"; i=0
for d in $DIRS; do
  i=$((i+1)); D="$WS/$d"; TAG=$(basename "$d")
  read SCEN IDX < <(python3 -c "import json;m=json.load(open('$D/run.json'));print(m['scenario'],m['run_index'])")
  say "($i/$N) $TAG"
  FR="$D/frames"; rm -rf "$FR"
  ./replay.sh --pose-log "$D/pose.csv" --crowd-log "$D/crowd.csv" --scene "$D/scene.json" \
    --out "$FR" --scenario "$SCEN" --run-index "$IDX" --fps 30 --width 1280 --height 720 > "$D/replay.log" 2>&1
  grep -E "^\[replay\] (完成|相機避牆)" "$D/replay.log"
  for cam in topdown chase oblique; do
    n=$(ls "$FR/$cam" 2>/dev/null | wc -l)
    if [ "$n" -lt 30 ]; then say "  ⚠ $cam 只有 $n 幀，保留舊影片"; continue; fi
    ffmpeg -y -nostdin -hide_banner -loglevel error -framerate 30 -i "$FR/$cam/rgb_%04d.png" \
      -c:v h264_nvenc -preset p5 -cq 23 -pix_fmt yuv420p -f mp4 "$D/video/${TAG}_${cam}.mp4.new" 2>>"$D/encode.log" \
      && mv -f "$D/video/${TAG}_${cam}.mp4.new" "$D/video/${TAG}_${cam}.mp4" || say "  ⚠ $cam 編碼失敗"
  done
  cp -f "$FR/frame_times.csv" "$D/frame_times.csv" 2>/dev/null; rm -rf "$FR"
done
say "收尾"
python3 scripts/make_sync.py recordings 2>&1 | grep -vE "^\[(WARN|INFO)\]" | tail -1
python3 scripts/check_recordings.py recordings > reports/recordings_health.txt 2>&1; tail -1 reports/recordings_health.txt
tmp=$(mktemp); python3 scripts/make_readme.py recordings > "$tmp" && [ -s "$tmp" ] && mv "$tmp" recordings/README.md && chmod 644 recordings/README.md && say "README 已更新"
python3 scripts/make_browse_tree.py >/dev/null 2>&1 && say "中文索引已重建"
say "補錄回放全部完成"
