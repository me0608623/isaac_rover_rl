"""定位誤差計算的測試。"""

from __future__ import annotations

import math

import pytest

from localization_error import compose_2d, interp_series, yaw_error_deg


def test_compose_identity():
    assert compose_2d((0.0, 0.0, 0.0), (3.0, 4.0, 0.5)) == pytest.approx((3.0, 4.0, 0.5))


def test_compose_applies_parent_rotation():
    """★ 串接要把子變換**轉到父座標系**再相加。
    直接把 x/y 相加是常見錯法，車一轉彎誤差就爆掉。"""
    got = compose_2d((0.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0))
    assert got[0] == pytest.approx(0.0, abs=1e-9)
    assert got[1] == pytest.approx(1.0)


def test_compose_adds_yaw():
    got = compose_2d((0.0, 0.0, 0.3), (0.0, 0.0, 0.4))
    assert got[2] == pytest.approx(0.7)


def test_yaw_error_takes_the_short_way():
    """★ 359° 與 1° 只差 2°，不是 358°。"""
    assert yaw_error_deg(math.radians(359), math.radians(1)) == pytest.approx(2.0, abs=1e-6)
    assert yaw_error_deg(math.radians(1), math.radians(359)) == pytest.approx(2.0, abs=1e-6)


def test_interp_series_between_samples():
    s = [(0.0, (0.0, 0.0, 0.0)), (1.0, (10.0, 0.0, 0.0))]
    assert interp_series(s, 0.25)[0] == pytest.approx(2.5)


def test_interp_series_clamps_outside():
    s = [(1.0, (5.0, 0.0, 0.0)), (2.0, (6.0, 0.0, 0.0))]
    assert interp_series(s, 0.0)[0] == pytest.approx(5.0)
    assert interp_series(s, 9.0)[0] == pytest.approx(6.0)


def test_interp_series_yaw_wraps_short_way():
    s = [(0.0, (0.0, 0.0, math.radians(350))), (1.0, (0.0, 0.0, math.radians(10)))]
    y = math.degrees(interp_series(s, 0.5)[2]) % 360
    assert min(abs(y - 0.0), abs(y - 360.0)) < 1.0, f"插出 {y:.1f}°，走了長弧"


def test_interp_series_empty_raises():
    """★ 空序列要報錯，不能回 (0,0,0) —— 那會變成「誤差剛好等於車的位置」，
    數字看起來有模有樣但完全是假的。"""
    with pytest.raises(ValueError):
        interp_series([], 0.0)
