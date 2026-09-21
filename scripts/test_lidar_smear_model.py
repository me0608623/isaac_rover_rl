"""掃描運動拖影模型的測試。

驗的是**物理量**：車端文件寫「0.8 m/s × 100 ms = 8 cm 畸變」，
這個數字必須從模型算得出來，否則模型沒有重現要模擬的誤差源。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from lidar_smear_model import smear_points

T_SCAN = 0.1          # VLP-16 @ rpm 600
V_TYPICAL = 0.8       # 車端實測巡航速度


def test_no_motion_means_no_smear():
    xyz = np.array([[10.0, 0, 0], [0, 5.0, 1.0], [-3.0, -4.0, -0.5]])
    out = smear_points(xyz, 0.0, 0.0, 0.0, T_SCAN)
    assert np.allclose(out, xyz)


def test_max_displacement_matches_vehicle_measurement():
    """車端 2026-08-24：0.8 m/s × 100 ms → 8 cm。這是模型的驗收量。"""
    phis = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    xyz = np.stack([10 * np.cos(phis), 10 * np.sin(phis), np.zeros_like(phis)], axis=1)
    out = smear_points(xyz, V_TYPICAL, 0.0, 0.0, T_SCAN)
    assert np.abs(out - xyz).max() == pytest.approx(V_TYPICAL * T_SCAN, abs=1e-9)


def test_scan_start_azimuth_gets_full_period():
    """φ=0 是掃描起點，離掃描結束最遠，位移量最大。"""
    out = smear_points(np.array([[10.0, 0.0, 0.0]]), V_TYPICAL, 0.0, 0.0, T_SCAN)
    assert out[0, 0] == pytest.approx(10.0 + V_TYPICAL * T_SCAN, abs=1e-9)


def test_rear_point_gets_half_period():
    """φ=π 時掃到一半，dt=T/2。

    正後方 10 m 的牆：測得時感測器在後方 4 cm 處 → 該牆當時只有 9.96 m 遠。
    """
    out = smear_points(np.array([[-10.0, 0.0, 0.0]]), V_TYPICAL, 0.0, 0.0, T_SCAN)
    assert out[0, 0] == pytest.approx(-10.0 + V_TYPICAL * T_SCAN / 2, abs=1e-9)


def test_displacement_grows_linearly_with_speed():
    xyz = np.array([[10.0, 0.0, 0.0]])
    d1 = smear_points(xyz, 0.4, 0, 0, T_SCAN)[0, 0] - 10.0
    d2 = smear_points(xyz, 0.8, 0, 0, T_SCAN)[0, 0] - 10.0
    assert d2 == pytest.approx(2 * d1, rel=1e-9)


def test_pure_rotation_displaces_by_arc_length():
    """純轉向時，遠處的點位移 ≈ r·ω·dt，這是走廊轉角最大的畸變來源。"""
    omega = 1.0
    out = smear_points(np.array([[10.0, 0.0, 0.0]]), 0, 0, omega, T_SCAN)
    assert math.hypot(out[0, 0] - 10.0, out[0, 1]) == pytest.approx(
        2 * 10.0 * math.sin(omega * T_SCAN / 2), rel=1e-9)


def test_z_is_untouched():
    """拖影是 2D 車體運動造成的，z 不應改變。"""
    xyz = np.array([[5.0, 3.0, 1.23], [-2.0, 7.0, -0.45]])
    out = smear_points(xyz, 0.8, 0.1, 0.5, T_SCAN)
    assert np.allclose(out[:, 2], xyz[:, 2])


def test_empty_cloud_is_safe():
    assert len(smear_points(np.zeros((0, 3)), 1.0, 0, 1.0, T_SCAN)) == 0


def test_static_vs_moving_ratio_is_large():
    """驗收表最關鍵的單一判準：靜止 vs 移動的擾動比要 >= 20 倍（實車 24 倍）。

    這裡驗的是拖影本身的量級差，不是 NDT 的 map->odom 跳幅
    （那要實際跑起來才量得到），但若拖影模型本身分不出靜止與移動，
    下游的 20 倍判準不可能成立。
    """
    phis = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    xyz = np.stack([10 * np.cos(phis), 10 * np.sin(phis), np.zeros_like(phis)], axis=1)
    still = np.abs(smear_points(xyz, 0.0, 0, 0, T_SCAN) - xyz).max()
    moving = np.abs(smear_points(xyz, V_TYPICAL, 0, 0, T_SCAN) - xyz).max()
    assert still == 0.0
    assert moving > 0.05
