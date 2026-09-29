"""即時速度資訊欄的測試（2026-09-25）。"""

from __future__ import annotations

import math

import numpy as np

from velocity_panel import velocities, yaw_from_quat


def test_straight_forward_motion_gives_positive_speed_zero_turn():
    t = np.arange(0, 3, 1 / 30)
    yaw = np.full_like(t, 0.7)
    x, y = 0.5 * t * math.cos(0.7), 0.5 * t * math.sin(0.7)
    v, w = velocities(t, x, y, yaw)
    assert abs(np.median(v) - 0.5) < 1e-6 and abs(np.median(w)) < 1e-6


def test_reversing_is_negative():
    """★ 車允許倒車：往車頭反方向移動要是負的，不能用速度大小。"""
    t = np.arange(0, 3, 1 / 30)
    yaw = np.zeros_like(t)
    v, _ = velocities(t, -0.2 * t, np.zeros_like(t), yaw)
    assert np.median(v) < -0.19


def test_turning_in_place_across_pi_is_smooth():
    """★ 朝向跨過 ±π 時不可出現 2π 的尖刺。"""
    t = np.arange(0, 4, 1 / 30)
    yaw = (3.0 + 0.4 * t + math.pi) % (2 * math.pi) - math.pi
    _, w = velocities(t, np.zeros_like(t), np.zeros_like(t), yaw)
    assert np.max(np.abs(w[5:-5] - 0.4)) < 1e-6


def test_yaw_from_quaternion():
    a = 1.1
    assert abs(yaw_from_quat(math.cos(a / 2), 0, 0, math.sin(a / 2)) - a) < 1e-9
