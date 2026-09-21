#!/bin/bash
# 啟動 Isaac Sim（模擬端）。ROS 環境要先備妥，Isaac 的 ros2 bridge 才連得上。
#
# 刻意清掉 PYTHONPATH：Jazzy 會把它指向 python3.12 的 site-packages，
# 而 Isaac Sim 跑的是 conda 的 python3.11，混進去會 import 到不相容的套件。
# Isaac 的 ROS bridge 是 C++（libisaacsim.ros2.bridge.jazzy.so），只需要 LD_LIBRARY_PATH。
cd "$(dirname "$0")"
export DISPLAY=${DISPLAY:-:1}   # GUI 模式需要
source ./setup_sim_env.sh >/dev/null
unset PYTHONPATH
exec env PYTHONUNBUFFERED=1 /home/aa/miniconda3/envs/env_isaaclab/bin/python scripts/run_isaac_sim.py "$@"
