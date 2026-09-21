#!/bin/bash
# 在 ROS 2 Jazzy 下編譯從車端 (Humble) rsync 過來的套件。
# 刻意不啟用 conda env_isaaclab —— 它的 Python 3.11 會與 Jazzy 的 3.12 衝突。
set -o pipefail
source /opt/ros/jazzy/setup.bash
cd /home/aa/IsaacLab/sim_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1
echo "EXIT=$?"
