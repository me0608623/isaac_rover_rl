#!/bin/bash
# sim_deploy_shell — 背景起棧 + 前景繁中 TUI 儀表板。對應車端的 deploy_rl_shell。
#
# 與 sim_deploy 的分工（沿用車端慣例）：
#   sim_deploy        = 純 ros2 launch（前景滾動 log），任何 shell 皆可，含非互動環境
#   sim_deploy_shell  = 本腳本，背景 launch + 前景 curses TUI，需真實 TTY
#
# curses 在非互動 shell（pipe / AI 工具的 Bash）會卡住或亂碼，故先守門。
cd "$(dirname "$0")"

if [ ! -t 0 ] || [ ! -t 1 ]; then
    echo "[sim_deploy_shell] 這是互動式 UI，需在真實終端機執行。"
    echo "  • 程式/AI 中啟動整棧 → 改用：  ./sim_deploy.sh"
    echo "  • 看即時狀態（可解析）→        ros2 topic echo /rover_rl_policy/status"
    echo "  • 停止整棧 →                   ./sim_deploy_stop.sh"
    exit 2
fi

# 啟動前檢查殘留
STALE=$(pgrep -f "ros2 launch launch/sim_deploy.launch.py|sim_ws/scripts/odom_drift_injector|ndt_localizer/lib" 2>/dev/null | wc -l)
if [ "$STALE" -gt 0 ]; then
    echo "┌─ ⚠ 偵測到 $STALE 個殘留節點（上次沒收乾淨）"
    echo "│ 不清會與新棧並存搶發 cmd_vel → 車走走停停。"
    echo "└──────────────────────────────────────────"
    read -rp "是否清除？[Y/n]（Enter=清除） " SEL
    case "$SEL" in [Nn]*) ;; *) ./sim_deploy_stop.sh ;; esac
fi

source ./setup_sim_env.sh
[ -d /home/aa/IsaacLab/rover_rl/install ] && source /home/aa/IsaacLab/rover_rl/install/setup.bash

if ! timeout 5 ros2 topic list 2>/dev/null | grep -q '^/clock$'; then
    echo "[sim_deploy_shell] ⚠ 收不到 /clock —— Isaac Sim 沒在跑？"
    echo "                   另開終端機執行： cd $PWD && ./run_sim.sh"
    read -rp "仍要繼續？[y/N] " GO
    case "$GO" in [Yy]*) ;; *) exit 1 ;; esac
fi

LOG="$PWD/log/sim_deploy_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$(dirname "$LOG")"
echo "[sim_deploy_shell] 背景啟動棧，log → $LOG"
ros2 launch launch/sim_deploy.launch.py enable_rviz:=true "$@" > "$LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() { echo; echo "[sim_deploy_shell] 收棧中…"; ./sim_deploy_stop.sh; }
trap cleanup EXIT INT TERM

echo "[sim_deploy_shell] 等待節點起來…"
sleep 8

if ros2 pkg executables rover_rl_inference 2>/dev/null | grep -q status_tui; then
    ros2 run rover_rl_inference status_tui --ros-args -p use_sim_time:=true
else
    echo "[sim_deploy_shell] 找不到 status_tui，改用 log 追蹤（Ctrl-C 收棧）"
    tail -f "$LOG"
fi
