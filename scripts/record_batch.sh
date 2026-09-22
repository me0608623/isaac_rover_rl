#!/usr/bin/env bash
# 論文錄影批次：3 情境 × 4 趟來回 × 3 視角 = 36 段影片，每趟同時錄一份 rosbag。
#
# ⚠ 一趟分**兩遍**跑，原因見 scripts/pose_log.py：
#   第一遍  正常速度跑導航（不算圖，RTF≈0.92）＋錄 rosbag ＋寫位姿 CSV
#   第二遍  不跑 ROS，照位姿 CSV 回放，用 path tracing 慢慢算圖
#   邊導航邊算圖是不行的：path tracing 把 RTF 壓到 0.35，實測 cmd_vel 被釘在
#   0.060 m/s（正常 0.475），200 模擬秒只走 14 m，到不了終點。
#
# 命名的唯一真相在 scripts/record_plan.py；影片與 bag 共用同一個 tag。
#
# ⚠ 刻意不用 `set -u`：ROS 的 setup.bash 會讀未定義變數，開了會直接死在 source。
# ⚠ 每趟結束就把 PNG 序列刪掉 —— 一趟 3 視角 × 30fps × 130 s ≈ 17 GB。
set -o pipefail
cd "$(dirname "$0")/.." || exit 1
WS=$PWD

ROOT=${ROOT:-$WS/recordings}
RUNS=${RUNS:-4}
LEG_TIMEOUT=${LEG_TIMEOUT:-200}   # 每段逾時（模擬秒；monitor 用 --sim-time）
WIDTH=${WIDTH:-1280}
HEIGHT=${HEIGHT:-720}
FPS=${FPS:-30}
#: 三個模型的 yaml 各有各的 speed_rate（sa4r2/sa4r3 0.6、sa5r2 0.7），
#: 直接比會被速度上限混淆。2026-09-22 使用者指定**統一 0.7**。
#: 走 launch 的覆寫參數，不動車端 yaml —— 那是實車契約，不該為了錄影改。
SPEED_RATE=${SPEED_RATE:-0.7}
BAG_TOPICS=(
    /clock /tf /tf_static
    /velodyne_points /filtered_points /ndt_points_map
    /ndt_pose /odom /odom_gt /cmd_vel
    /global_path /goal_pose /initialpose
    /vo_safety_node/status /rover_rl_policy/status
    /charge_description
)

mkdir -p "$ROOT"
BATCH_LOG="$ROOT/batch.log"
say () { echo "[$(date '+%F %T')] $*" | tee -a "$BATCH_LOG"; }

cleanup_all () {
    ./sim_deploy_stop.sh >/dev/null 2>&1
    pkill -f "ros2 ba[g] record" 2>/dev/null
    pkill -INT -f "run_isaac_si[m].py" 2>/dev/null
    pkill -INT -f "replay_rende[r].py" 2>/dev/null
    sleep 6
    pkill -9 -f "run_isaac_si[m].py" 2>/dev/null
    pkill -9 -f "replay_rende[r].py" 2>/dev/null
    sleep 2
}
trap 'say "收到中斷，收拾現場"; cleanup_all; exit 130' INT TERM

PLAN_FILE=$(mktemp)
PYTHONPATH="$WS/scripts" .venv/bin/python - "$RUNS" "$ROOT" > "$PLAN_FILE" <<'PY'
import sys
from record_plan import build_plan
for r in build_plan(int(sys.argv[1]), sys.argv[2]):
    print(f"{r.model}\t{r.scenario}\t{r.tag}\t{r.run_index}")
PY
if [ ! -s "$PLAN_FILE" ]; then say "⚠ 產不出計畫表，中止"; exit 2; fi
say "計畫共 $(wc -l < "$PLAN_FILE") 趟來回 → $ROOT"

# ── 第一遍：導航 + rosbag + 位姿 CSV ───────────────────────────────────
pass_one () {
    local MODEL="$1" SCEN="$2" TAG="$3" DIR="$4" IDX="$5"
    local BAG="$DIR/bag" NAV="$DIR/nav" POSE="$DIR/pose.csv"
    local CROWD="$DIR/crowd.csv"
    cleanup_all

    say "  [1a] Isaac（不算圖，全速）"
    nohup ./run_sim.sh --scenario "$SCEN" --run-index "$IDX" \
          --pose-log "$POSE" --crowd-log "$CROWD" \
          > "$DIR/isaac_nav.log" 2>&1 &
    local pid=$! ok=0
    for _ in $(seq 1 80); do
        grep -q "開始模擬" "$DIR/isaac_nav.log" 2>/dev/null && { ok=1; break; }
        kill -0 "$pid" 2>/dev/null || break
        sleep 5
    done
    [ "$ok" = 1 ] || { say "  ⚠ Isaac 沒起來"; return 1; }

    say "  [1b] ROS 棧（policy + VO，不開 RViz）"
    . ./setup_sim_env.sh >/dev/null 2>&1
    . /home/aa/IsaacLab/rover_rl/install/setup.bash >/dev/null 2>&1
    nohup ros2 launch launch/sim_deploy.launch.py enable_rviz:=false \
          rl_profile:="$MODEL" speed_rate:="$SPEED_RATE" \
          > "$DIR/stack.log" 2>&1 &
    for _ in $(seq 1 60); do
        timeout 5 ros2 topic list 2>/dev/null | grep -q '^/ndt_pose$' && break
        sleep 4
    done
    sleep 6
    # ⚠ 一定要回頭確認**實際載入的**是哪一顆。只看自己傳了什麼參數，
    #   profile 名打錯或 launch 預設沒吃到，會安靜地用預設模型錄完 12 趟。
    local GOT
    GOT=$(grep -m1 "RL profile" "$DIR/stack.log" 2>/dev/null)
    case "$GOT" in
        *"'$MODEL'"*) say "  [1b'] 模型確認：$GOT" ;;
        *) say "  ⚠ 模型不符！要求 $MODEL，實際 ${GOT:-讀不到}"; return 1 ;;
    esac

    # ⚠ 同理要驗**實際生效**的 speed_rate，不是自己傳了什麼。
    #   policy 會把它發在 /rover_rl_policy/status 裡，直接讀那個。
    local SR
    SR=$(timeout 25 ros2 topic echo --once /rover_rl_policy/status 2>/dev/null \
         | grep -oP '"speed_rate":\s*\K[0-9.]+' | head -1)
    case "$SR" in
        "$SPEED_RATE") say "  [1b2] speed_rate 生效值：$SR" ;;
        "") say "  ⚠ 讀不到 speed_rate 生效值（policy status 沒出來）"; return 1 ;;
        *) say "  ⚠ speed_rate 不符！要求 $SPEED_RATE，實際 $SR"; return 1 ;;
    esac

    say "  [1c] 初始位姿 c28"
    timeout 30 python3 scripts/publish_initial_pose.py --ros-args \
        -p node_name:=c28 -p use_sim_time:=true > "$DIR/initpose.log" 2>&1
    sleep 8

    say "  [1d] 開始錄 rosbag"
    nohup ros2 bag record -o "$BAG/$TAG" \
          --compression-mode file --compression-format zstd \
          "${BAG_TOPICS[@]}" > "$DIR/bag.log" 2>&1 &
    sleep 4

    say "  [1e] 導航 c28 → c25 → c28"
    timeout 2400 python3 scripts/monitor_navigation.py \
        --legs c25,c28 --start c28 --sim-time --seconds "$LEG_TIMEOUT" \
        --log-dir "$NAV" --tag "$TAG" 2>&1 \
        | grep -vE "^\[WARN\]|deprecated|localhost" > "$DIR/nav.log"
    sed -n '/^段  /,$p' "$DIR/nav.log" | head -5 | tee -a "$BATCH_LOG"

    pkill -INT -f "ros2 ba[g] record" 2>/dev/null
    sleep 5
    ./sim_deploy_stop.sh >/dev/null 2>&1
    pkill -INT -f "run_isaac_si[m].py" 2>/dev/null
    for _ in $(seq 1 24); do
        pgrep -f "run_isaac_si[m].py" >/dev/null || break
        sleep 5
    done
    pkill -9 -f "run_isaac_si[m].py" 2>/dev/null
    sleep 3
    [ -s "$POSE" ] || { say "  ⚠ 位姿 CSV 是空的"; return 1; }
    say "  [1f] 位姿軌跡 $(wc -l < "$POSE") 筆"
}

# ── 第二遍：回放算圖 + 編碼 ────────────────────────────────────────────
pass_two () {
    local SCEN="$1" TAG="$2" DIR="$3" IDX="$4"
    # 第二遍不跑 ROS，模型與它無關（只照第一遍的位姿回放）。
    local FRAMES="$DIR/frames" VIDEO="$DIR/video"
    say "  [2a] 回放算圖（path tracing）"
    ./replay.sh --pose-log "$DIR/pose.csv" --crowd-log "$DIR/crowd.csv" \
        --out "$FRAMES" --scenario "$SCEN" --run-index "$IDX" --fps "$FPS" \
        --width "$WIDTH" --height "$HEIGHT" > "$DIR/replay.log" 2>&1
    grep -E "^\[replay\] (第一幀|完成|情境)" "$DIR/replay.log" | tee -a "$BATCH_LOG"

    say "  [2b] 編碼 3 段影片"
    local cam n
    for cam in topdown chase oblique; do
        n=$(ls "$FRAMES/$cam" 2>/dev/null | wc -l)
        if [ "$n" -lt 30 ]; then say "    ⚠ $cam 只有 $n 幀，跳過"; continue; fi
        ffmpeg -y -nostdin -hide_banner -loglevel error -framerate "$FPS" \
            -i "$FRAMES/$cam/rgb_%04d.png" \
            -c:v h264_nvenc -preset p5 -cq 23 -pix_fmt yuv420p \
            "$VIDEO/${TAG}_${cam}.mp4" 2>>"$DIR/encode.log" \
            && say "    $cam: $n 幀 → $(du -h "$VIDEO/${TAG}_${cam}.mp4" | cut -f1)" \
            || say "    ⚠ $cam 編碼失敗，見 $DIR/encode.log"
    done
    cp -f "$FRAMES/frame_times.csv" "$DIR/frame_times.csv" 2>/dev/null
    rm -rf "$FRAMES"
}

run_one () {
    local MODEL="$1" SCEN="$2" TAG="$3" IDX="$4"
    local DIR="$ROOT/$TAG"
    if [ -s "$DIR/video/${TAG}_chase.mp4" ] && [ -f "$DIR/run.json" ]; then
        say "  $TAG 已完成，跳過"; return 0
    fi
    say "════ $TAG（模型 $MODEL／情境 $SCEN）════"
    rm -rf "$DIR"; mkdir -p "$DIR/bag" "$DIR/nav" "$DIR/video"
    pass_one "$MODEL" "$SCEN" "$TAG" "$DIR" "$IDX" \
        || { say "  ⚠ $TAG 第一遍失敗，跳過"; cleanup_all; return 1; }
    pass_two "$SCEN" "$TAG" "$DIR" "$IDX"

    PYTHONPATH="$WS/scripts" .venv/bin/python - "$DIR" "$SCEN" "$TAG" "$FPS" "$MODEL" "$SPEED_RATE" "$IDX" <<'PY'
import json, sys
from pathlib import Path
d, scen, tag, fps, model, srate = (Path(sys.argv[1]), sys.argv[2], sys.argv[3],
                                   int(sys.argv[4]), sys.argv[5], float(sys.argv[6]))
nav = (d / "nav.log").read_text(errors="replace") if (d / "nav.log").exists() else ""
meta = {
    "tag": tag, "model": model, "scenario": scen, "fps": fps,
    "speed_rate": srate,
    "route": ["c28", "c25", "c28"],
    "model_loaded": next((l.strip() for l in
                          (d / "stack.log").read_text(errors="replace").splitlines()
                          if "RL profile" in l), None) if (d / "stack.log").exists() else None,
    "videos": sorted(p.name for p in (d / "video").glob("*.mp4")),
    "bag": sorted(p.name for p in (d / "bag").rglob("*.mcap")),
    "nav_csv": sorted(p.name for p in (d / "nav").glob("*.csv")),
    "pose_rows": sum(1 for _ in (d / "pose.csv").open()) - 1
                 if (d / "pose.csv").exists() else 0,
    "crowd_rows": sum(1 for _ in (d / "crowd.csv").open()) - 1
                  if (d / "crowd.csv").exists() else 0,
    "crowd_mode": "orca",
    "run_index": int(__import__("sys").argv[7]) if len(__import__("sys").argv) > 7 else 0,
    "nav_summary": [l.rstrip() for l in nav.splitlines()
                    if l.strip() and not l.startswith("[")],
    "note": ("兩遍錄製：第一遍全速導航並錄 rosbag/位姿，第二遍照位姿回放算圖。"
             "影片幀率＝模擬時間 30 fps；frame_times.csv 給幀號↔模擬時間，"
             "rosbag 用 use_sim_time 錄，兩者靠模擬時間對齊。"),
}
(d / "run.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2))
PY
    say "  $TAG 完成"
}

# ⚠ 迴圈的輸入走 **FD 3**，不要用 stdin。
#   2026-09-22 踩過：ffmpeg 預設會把 stdin 整個吃掉，計畫表被讀光，
#   12 趟只跑了第 1 趟就印「批次完成」，而且沒有任何錯誤訊息。
#   ffmpeg 那邊也補了 -nostdin，兩道保險。
while IFS=$'\t' read -r MODEL SCEN TAG IDX <&3; do
    [ -z "$TAG" ] && continue
    if [ -n "${ONLY:-}" ] && [[ "$TAG" != *"$ONLY"* ]]; then continue; fi
    run_one "$MODEL" "$SCEN" "$TAG" "$IDX"
done 3< "$PLAN_FILE"

cleanup_all
say "════ 批次完成：$(ls -d "$ROOT"/*/ 2>/dev/null | wc -l) 個目錄，$(find "$ROOT" -name '*.mp4' | wc -l) 段影片 ════"
