"""讓 pytest 直接找得到同目錄的模組。

測試用的 python 是 sim_ws/.venv/bin/python（含 usd-core），
刻意與 conda env_isaaclab 隔離 —— Isaac Sim 自帶一套 pxr，
在 env_isaaclab 裝 usd-core 有覆蓋它的風險。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
