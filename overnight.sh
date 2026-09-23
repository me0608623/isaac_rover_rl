#!/usr/bin/env bash
# overnight.sh — 夜間排程：等主批次跑完，接著做完收尾分析與三個對照組。
#
# 每一階段都寫進 reports/overnight.log，任何一階段失敗只記錄、不中斷後面。
# ⚠ 刻意不用 set -u（ROS 的 setup.bash 會讀未定義變數）。
set -o pipefail
cd "$(dirname "$0")" || exit 1
WS=$PWD
mkdir -p reports
LOG=$WS/reports/overnight.log
say () { echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
stage () { say "═══ $* ═══"; }

say "夜間排程啟動"

# ── 0. 等主批次 ───────────────────────────────────────────────────────
stage "0/8 等主批次結束"
while pgrep -f "record_batch\.sh" >/dev/null 2>&1; do sleep 120; done
say "主批次已結束：$(ls -d "$WS"/recordings/*/ 2>/dev/null | grep -vc '_v[12]_') 趟"

source "$WS/setup_sim_env.sh" >/dev/null 2>&1

# ── 1~4. 收尾分析（只吃 CPU）────────────────────────────────────────
stage "1/8 影片↔rosbag 對齊偏移寫進 run.json"
python3 scripts/make_sync.py recordings 2>&1 | grep -vE "^\[(WARN|INFO)\]" | tee -a "$LOG"

stage "2/8 定位誤差全量"
python3 scripts/localization_error.py recordings 2>&1 \
    | grep -vE "^\[(WARN|INFO)\]" | tee reports/localization_error.txt | tail -20 | tee -a "$LOG"

stage "3/8 影片健檢（抓全黑與幀數不對）"
python3 scripts/check_recordings.py recordings > reports/recordings_health.txt 2>&1
tail -6 reports/recordings_health.txt | tee -a "$LOG"

stage "4/8 產生結果 README"
python3 scripts/make_readme.py recordings > recordings/README.md 2>>"$LOG" \
    && say "  recordings/README.md（$(wc -l < recordings/README.md) 行）"

# ── 5. 給 record_batch.sh 打補丁（批次已結束，現在可以改）──────────
stage "5/8 record_batch.sh 補丁（CROWD_MODE + rosbag 收尾等待）"
python3 scripts/patch_record_batch.py scripts/record_batch.sh 2>&1 | tee -a "$LOG"
bash -n scripts/record_batch.sh && say "  語法 OK" || { say "  ⚠ 語法壞了，跳過對照組"; exit 1; }

# ── 6~8. 三個對照組（各 12 趟 / 36 段，只用 sa4r2）─────────────────
run_arm () {
    local NAME="$1" ; shift
    stage "$NAME"
    # ⚠ 必須用 env：`A=1 "$@" bash ...` 這種寫法裡，"$@" 展開出來的
    #   `CROWD_MODE=path` 會被當成**命令名**（assignment 前綴只認字面值），
    #   2026-09-23 實測報 "CROWD_MODE=path：指令找不到"，三個對照組全都
    #   沒跑卻印「完成 0 段」。env 會把 VAR=VAL 當參數正確處理。
    env ROOT="$WS/recordings_abl/$NAME" ONLY=sa4r2 "$@" \
        bash scripts/record_batch.sh >> "$WS/reports/abl_$NAME.log" 2>&1
    local n; n=$(find "$WS/recordings_abl/$NAME" -name '*.mp4' 2>/dev/null | wc -l)
    if [ "$n" -lt 3 ]; then
        say "  ⚠ $NAME 只產出 $n 段影片 —— 這一組失敗了，見 reports/abl_$NAME.log"
        tail -5 "$WS/reports/abl_$NAME.log" | tee -a "$LOG"
        return 1
    fi
    say "  $NAME 完成：$n 段影片"
    python3 scripts/make_sync.py "recordings_abl/$NAME" 2>&1 | grep -c 已寫入 >/dev/null
    python3 scripts/localization_error.py "recordings_abl/$NAME" \
        > "reports/localization_error_$NAME.txt" 2>&1
    python3 scripts/make_readme.py "recordings_abl/$NAME" \
        > "$WS/recordings_abl/$NAME/README.md" 2>/dev/null
}

run_arm "crowd_path"  CROWD_MODE=path
run_arm "speed_0p6"   SPEED_RATE=0.6
run_arm "speed_1p0"   SPEED_RATE=1.0

say "═══ 夜間排程全部完成 ═══"
say "主批次 $(find "$WS/recordings" -name '*.mp4' -not -path '*_v[12]_*' | wc -l) 段"
for a in crowd_path speed_0p6 speed_1p0; do
    say "  對照組 $a：$(find "$WS/recordings_abl/$a" -name '*.mp4' 2>/dev/null | wc -l) 段"
done
