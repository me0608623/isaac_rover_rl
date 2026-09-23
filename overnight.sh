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

# ── 0. 主批次 ─────────────────────────────────────────────────────────
#
# ⚠⚠ 2026-09-23：這一步原本**只會「等」**主批次，不會啟動它。於是直接
#   `bash overnight.sh` 會在沒人跑主批次的情況下「等到 0 秒就結束」，
#   對著空的 recordings/ 做完收尾分析（全部 0 趟、不報錯），然後直接跳去
#   跑對照組 —— 主批次整個被跳過。改成沒在跑就自己啟動。
stage "0/8 主批次"
if pgrep -f "record_batch\.sh" >/dev/null 2>&1; then
    say "  已有主批次在跑，等它結束"
else
    say "  沒有主批次在跑 → 現在啟動"
    bash scripts/record_batch.sh >> "$WS/reports/abl_main.log" 2>&1 &
    sleep 10
fi
while pgrep -f "record_batch\.sh" >/dev/null 2>&1; do sleep 120; done
_N_MAIN=$(find "$WS/recordings" -name run.json -not -path '*/00_*' -not -path '*/_*' | wc -l)
say "主批次已結束：$_N_MAIN 趟"
if [ "$_N_MAIN" -lt 30 ]; then
    say "  ⚠ 主批次只有 $_N_MAIN 趟（預期 36）—— 見 reports/abl_main.log"
fi

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
# ⚠ 先寫到暫存檔、成功才覆蓋。直接 `> README.md` 的話 shell 會先把它清空，
#   主批次失敗（0 趟）時 make_readme 回傳 1、什麼都沒印 —— README 就被抹成空檔。
write_readme () {
    local root="$1" tmp
    tmp=$(mktemp)
    if python3 scripts/make_readme.py "$root" > "$tmp" 2>>"$LOG" && [ -s "$tmp" ]; then
        mv "$tmp" "$root/README.md"
        say "  $root/README.md（$(wc -l < "$root/README.md") 行）"
    else
        rm -f "$tmp"
        say "  ⚠ $root 產不出 README，保留舊的"
    fi
}
write_readme recordings
python3 scripts/make_browse_tree.py >> "$LOG" 2>&1 && say "  中文索引已重建"

# ── 5. 給 record_batch.sh 打補丁（批次已結束，現在可以改）──────────
stage "5/8 record_batch.sh 補丁（CROWD_MODE + rosbag 收尾等待）"
python3 scripts/patch_record_batch.py scripts/record_batch.sh 2>&1 | tee -a "$LOG"
bash -n scripts/record_batch.sh && say "  語法 OK" || { say "  ⚠ 語法壞了，跳過對照組"; exit 1; }

# ── 6~8. 三個對照組（各 12 趟 / 36 段，只用 sa4r2）─────────────────
# 對照組的資料夾名（中文）。唯一定義在 scripts/browse_names.py 的 ARM_DIR；
# 這裡問它一次，避免兩邊各寫一份而哪天對不上。
arm_dir () {
    PYTHONPATH="$WS/scripts" .venv/bin/python -c \
        "from browse_names import ARM_DIR; print(ARM_DIR['$1'])"
}

run_arm () {
    local NAME="$1" ; shift
    local DIR; DIR="$WS/recordings_abl/$(arm_dir "$NAME")"
    stage "$NAME"
    # ⚠ 必須用 env：`A=1 "$@" bash ...` 這種寫法裡，"$@" 展開出來的
    #   `CROWD_MODE=path` 會被當成**命令名**（assignment 前綴只認字面值），
    #   2026-09-23 實測報 "CROWD_MODE=path：指令找不到"，三個對照組全都
    #   沒跑卻印「完成 0 段」。env 會把 VAR=VAL 當參數正確處理。
    env ROOT="$DIR" ONLY=sa4r2 "$@" \
        bash scripts/record_batch.sh >> "$WS/reports/abl_$NAME.log" 2>&1
    local n; n=$(find "$DIR" -name '*.mp4' 2>/dev/null | wc -l)
    if [ "$n" -lt 3 ]; then
        say "  ⚠ $NAME 只產出 $n 段影片 —— 這一組失敗了，見 reports/abl_$NAME.log"
        tail -5 "$WS/reports/abl_$NAME.log" | tee -a "$LOG"
        return 1
    fi
    say "  $NAME 完成：$n 段影片"
    python3 scripts/make_sync.py "$DIR" 2>&1 | grep -c 已寫入 >/dev/null
    python3 scripts/localization_error.py "$DIR" \
        > "reports/localization_error_$NAME.txt" 2>&1
    write_readme "$DIR"
}

run_arm "crowd_path"  CROWD_MODE=path
run_arm "speed_0p6"   SPEED_RATE=0.6
run_arm "speed_1p0"   SPEED_RATE=1.0

say "═══ 夜間排程全部完成 ═══"
say "主批次 $(find "$WS/recordings" -name '*.mp4' -not -path '*_v[12]_*' | wc -l) 段"
for a in crowd_path speed_0p6 speed_1p0; do
    say "  對照組 $a：$(find "$WS/recordings_abl/$(arm_dir "$a")" -name '*.mp4' 2>/dev/null | wc -l) 段"
done
