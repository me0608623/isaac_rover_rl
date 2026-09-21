#!/bin/bash
# 跑 sim_ws 的所有測試。用專用 venv，不需要 Isaac Sim、不需要 ROS。
#
# 刻意清掉 PYTHONPATH 並關閉 pytest 外掛自動載入：
# shell 若 source 過 /opt/ros/jazzy，ROS 的 launch_testing 外掛會被自動載入，
# 它的 hook 簽章與新版 pytest 不相容，會讓整個測試無法啟動。
cd "$(dirname "$0")"
env -u PYTHONPATH -u AMENT_PREFIX_PATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    .venv/bin/python -m pytest scripts/ "$@"
