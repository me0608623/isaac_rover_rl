"""VLP-16 掃描運動拖影的數學模型（純 numpy，無 ROS 相依）。

與 lidar_motion_smear.py 的分工：
  - 本檔：模型本體，可在沒有 ROS 的環境 import 與單元測試
  - lidar_motion_smear.py：ROS 節點外殼

模型推導與已知限制見 lidar_motion_smear.py 的 docstring。
"""

from __future__ import annotations

import math

import numpy as np


def smear_points(xyz: np.ndarray, vx: float, vy: float, omega: float,
                 scan_period: float, azimuth_start: float = 0.0) -> np.ndarray:
    """對點雲施加掃描期間的運動拖影。純函式，可離線測試。

    Args:
        xyz: (N, 3) 感測器座標系下的理想快照（對應掃描結束時刻）。
        vx, vy: 感測器座標系下的線速度 (m/s)。
        omega: 角速度 (rad/s)，繞 +Z。
        scan_period: 一圈的時間 (s)，VLP-16 @600rpm = 0.1。
        azimuth_start: 掃描起始方位角 (rad)。

    Returns:
        (N, 3) 失真後的點雲。
    """
    if len(xyz) == 0:
        return xyz
    phi = np.arctan2(xyz[:, 1], xyz[:, 0])
    frac = np.mod(phi - azimuth_start, 2.0 * math.pi) / (2.0 * math.pi)
    dt = scan_period * (1.0 - frac)              # 越早掃到的點，dt 越大

    c, s = np.cos(omega * dt), np.sin(omega * dt)
    out = np.empty_like(xyz)
    # p_smear = R(omega*dt) @ p + v*dt   （column-vector 慣例，這裡是自己的數學不是 USD）
    out[:, 0] = c * xyz[:, 0] - s * xyz[:, 1] + vx * dt
    out[:, 1] = s * xyz[:, 0] + c * xyz[:, 1] + vy * dt
    out[:, 2] = xyz[:, 2]
    return out
