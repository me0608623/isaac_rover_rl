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


def test_oriented_rect_stays_counter_clockwise_for_any_yaw():
    """★★ RVO2 的障礙繞反了 agent 會被推進障礙裡（等於穿牆），而且不報錯。
    旋轉後必須仍是逆時針。"""
    import math

    from orca_crowd import oriented_rect, polygon_area

    for yaw in (0.0, 0.3, 1.57, 3.0, -2.2, -3.14):
        v = oriented_rect(1.0, 2.0, 0.9, 0.2, yaw)
        assert polygon_area(v) == pytest.approx(4 * 0.9 * 0.2)


def test_oriented_rect_matches_ccw_rect_at_zero_yaw():
    from orca_crowd import ccw_rect, oriented_rect

    a = oriented_rect(1.0, 2.0, 0.5, 0.3, 0.0)
    b = ccw_rect(1.0, 2.0, 0.5, 0.3)
    for p, q in zip(a, b):
        assert p == pytest.approx(q)


def test_oriented_rect_rotates_the_long_side():
    import math

    from orca_crowd import oriented_rect

    v = oriented_rect(0.0, 0.0, 1.0, 0.1, math.pi / 2)   # 長邊轉到 y 方向
    ys = [p[1] for p in v]
    xs = [p[0] for p in v]
    assert max(ys) - min(ys) == pytest.approx(2.0)
    assert max(xs) - min(xs) == pytest.approx(0.2)


# ── 朝向平滑（2026-09-24：行人快停下時一格轉 180°）──────────────────────

def test_turn_is_rate_limited_not_a_snap():
    import math
    from orca_crowd import turn_toward
    y = turn_toward(0.0, math.pi, 0.1)
    assert abs(y - 0.1) < 1e-9 or abs(y + 0.1) < 1e-9


def test_turn_takes_the_short_way_across_pi():
    import math
    from orca_crowd import turn_toward
    y = turn_toward(3.0, -3.0, 0.1)          # 真正差 0.28 rad，往正向繞過 π
    assert y > 3.0


def test_small_turn_reaches_target():
    from orca_crowd import turn_toward
    assert turn_toward(0.0, 0.05, 0.1) == 0.05


def test_none_target_keeps_facing_and_first_value_is_taken():
    from orca_crowd import turn_toward
    assert turn_toward(1.2, None, 0.1) == 1.2
    assert turn_toward(None, 0.7, 0.1) == 0.7


def test_back_and_forth_velocity_does_not_flip_the_body():
    """★ ORCA 快停下時速度方向前後翻轉：身體朝向每步最多動 max_step。"""
    import math
    from orca_crowd import turn_toward
    y, ys = 0.0, []
    for i in range(30):
        y = turn_toward(y, 0.0 if i % 2 else math.pi, 2.5 / 60.0)
        ys.append(y)
    assert max(abs(b - a) for a, b in zip(ys, ys[1:])) <= 2.5 / 60.0 + 1e-9
