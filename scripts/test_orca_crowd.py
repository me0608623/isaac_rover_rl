"""行人 ORCA 群體的純幾何測試（不需要 rvo2，故不碰 OrcaCrowd 本身）。"""

from __future__ import annotations

import math

import pytest

from orca_crowd import ccw_rect, next_goal_index, pref_velocity


def test_goal_flips_when_arrived():
    """★ 到了就換下一個目標，否則行人會停在終點不動。"""
    wp = ((0.0, 0.0), (10.0, 0.0))
    assert next_goal_index(1, (9.8, 0.0), wp, tol=0.8) == 0
    assert next_goal_index(0, (0.2, 0.0), wp, tol=0.8) == 1


def test_goal_holds_while_still_far():
    wp = ((0.0, 0.0), (10.0, 0.0))
    assert next_goal_index(1, (5.0, 0.0), wp, tol=0.8) == 1


def test_goal_does_not_chatter_at_the_boundary():
    """★ 剛好在容差邊緣時不能每一步都翻，否則行人會在原地抖。"""
    wp = ((0.0, 0.0), (10.0, 0.0))
    idx = 1
    for d in (0.79, 0.81, 0.79):          # 在邊界附近來回
        idx = next_goal_index(idx, (10.0 - d, 0.0), wp, tol=0.8)
    assert idx == 0, "翻過去之後就該朝另一端走，不該再翻回來"


def test_pref_velocity_points_at_the_goal_with_the_right_speed():
    vx, vy = pref_velocity((0.0, 0.0), (3.0, 4.0), speed=1.0)
    assert math.hypot(vx, vy) == pytest.approx(1.0)
    assert vx == pytest.approx(0.6) and vy == pytest.approx(0.8)


def test_pref_velocity_is_zero_on_top_of_the_goal():
    """★ 站在目標上時方向無定義，要回 0 而不是 NaN ——
    NaN 餵進 ORCA 會讓整個 solver 的輸出變成 NaN，行人瞬移到天邊。"""
    vx, vy = pref_velocity((1.0, 1.0), (1.0, 1.0), speed=1.0)
    assert (vx, vy) == (0.0, 0.0)


def test_ccw_rect_is_counter_clockwise():
    """★ RVO2 的靜態障礙要逆時針繞，順時針會變成「agent 只能待在裡面」，
    行人會被推穿牆。用有號面積驗證繞向。"""
    v = ccw_rect(0.0, 0.0, 1.0, 2.0)
    area2 = sum(v[i][0] * v[(i + 1) % 4][1] - v[(i + 1) % 4][0] * v[i][1]
                for i in range(4))
    assert area2 > 0, "有號面積為負 = 順時針"
    assert len(v) == 4


def test_ccw_rect_covers_the_requested_extent():
    v = ccw_rect(5.0, -2.0, 0.5, 1.5)
    xs = [p[0] for p in v]
    ys = [p[1] for p in v]
    assert min(xs) == pytest.approx(4.5) and max(xs) == pytest.approx(5.5)
    assert min(ys) == pytest.approx(-3.5) and max(ys) == pytest.approx(-0.5)
