# 模擬棧的統一 ROS 環境。**每個要跟模擬互動的終端機都要 source 這支。**
#
# 為什麼要統一：
#   RMW 不同的節點完全看不到彼此，而且不會報任何錯 —— 只會安靜地收不到 topic。
#   Isaac Sim 的 ROS 2 bridge 內建的 rmw **只有 FastDDS**（無 CycloneDDS），
#   所以整條鏈統一用 rmw_fastrtps_cpp，這在 Jazzy 也是現成的。
#
#   注意：車端用的是 ROS_DOMAIN_ID=55 + rmw_zenoh_cpp，與這裡不同。
#   兩邊本來就不該互通（模擬與實車各自獨立），但別把兩邊的設定混用。
#
# 用法：  source /home/aa/IsaacLab/sim_ws/setup_sim_env.sh

source /opt/ros/jazzy/setup.bash
if [ -f "/home/aa/IsaacLab/sim_ws/install/setup.bash" ]; then
    source /home/aa/IsaacLab/sim_ws/install/setup.bash
fi

export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

export SIM_WS=/home/aa/IsaacLab/sim_ws
export SIM_USD="$SIM_WS/assets/3floor_ver_1_ros_fixed.usda"

echo "[sim_env] ROS $ROS_DISTRO  DOMAIN=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
echo "[sim_env] USD = $SIM_USD"
