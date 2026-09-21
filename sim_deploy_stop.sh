#!/bin/bash
# sim_deploy_stop — 收掉模擬端 ROS 棧（不碰 Isaac Sim 本身）。
#
# 用「安裝路徑 / 腳本路徑」偵測而非節點名清單：以後新增節點自動涵蓋，
# 不必再維護第三份清單（車端就因為漏加清單留過孤兒，兩隻 recovery_supervisor
# 各自 10 Hz 灌 cmd_vel，車走走停停查了整輪才抓到）。
PATTERNS=(
    "ros2 launch launch/sim_deploy.launch.py"
    "sim_ws/scripts/odom_drift_injector.py"
    "sim_ws/scripts/lidar_motion_smear.py"
    "sim_ws/scripts/publish_initial_pose.py"
    "ndt_localizer/lib/ndt_localizer/"
    "campusrover_routing/lib/"
    "campusrover_demo/lib/campusrover_demo/simple_map_publisher"
    "rover_rl_inference/lib/rover_rl_inference/"
    "rviz2 -d .*sim_deploy.rviz"
    "robot_state_publisher"
    # ⚠ 2026-09-21 踩過：漏掉這條，多次重啟後累積了 6 個孤兒 world_to_map，
    #   各自發 world→map static TF。加這條之後才收得乾淨。
    "static_transform_publisher.*world.*map"
)
n=0
for pat in "${PATTERNS[@]}"; do
    for pid in $(pgrep -f "$pat" 2>/dev/null); do
        [ "$pid" = "$$" ] && continue
        kill -TERM "$pid" 2>/dev/null && n=$((n+1))
    done
done
sleep 2
for pat in "${PATTERNS[@]}"; do pkill -9 -f "$pat" 2>/dev/null; done
echo "[sim_deploy_stop] 已送出停止訊號給 $n 個程序（Isaac Sim 未受影響）"
