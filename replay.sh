#!/bin/bash
# 第二趟回放算圖（見 scripts/replay_render.py）。不需要 ROS，但沿用同一套環境。
cd "$(dirname "$0")"
source ./setup_sim_env.sh >/dev/null 2>&1
unset PYTHONPATH
exec env PYTHONUNBUFFERED=1 /home/aa/miniconda3/envs/env_isaaclab/bin/python \
     scripts/replay_render.py "$@"
