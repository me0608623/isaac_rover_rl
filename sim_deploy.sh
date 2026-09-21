#!/bin/bash
# sim_deploy — 模擬端 ROS 棧（前景，log 直接滾動）。對應車端的 deploy_rl。
#
# 前提：Isaac Sim 要先跑起來（另一個終端機執行 ./run_sim.sh），
#       否則 /clock、/velodyne_points_ideal、/odom_gt 都不存在，NDT 會空等。
#
# 用法：  ./sim_deploy.sh                       全開
#         ./sim_deploy.sh enable_policy:=false  只跑定位（先確認 NDT 收斂再開 policy）
#         ./sim_deploy.sh enable_rviz:=false    不開 RViz
cd "$(dirname "$0")"
source ./setup_sim_env.sh
if [ -d /home/aa/IsaacLab/rover_rl/install ]; then
    source /home/aa/IsaacLab/rover_rl/install/setup.bash
fi

if ! timeout 5 ros2 topic list 2>/dev/null | grep -q '^/clock$'; then
    echo "[sim_deploy] ⚠ 收不到 /clock —— Isaac Sim 可能沒在跑。"
    echo "             先在另一個終端機執行：  cd $PWD && ./run_sim.sh"
    echo "             仍要繼續請按 Enter，取消按 Ctrl-C。"
    read -r _
fi

exec ros2 launch launch/sim_deploy.launch.py "$@"
