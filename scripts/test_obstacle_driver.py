"""移動障礙物執行期驅動的測試（純計算部分，不需要 Isaac）。"""

from __future__ import annotations

import math

import pytest

import ros_graph_spec as S
from obstacle_driver import world_pose_at

FLOOR_TOP = -0.3233        # measure_corridor_floor_top 的實測值


def _walker(**kw):
    base = dict(name="w", waypoints=((-5.0, 4.0), (-5.0, 7.0)), speed=1.0,
                mode="pingpong", phase_s=0.0, height=1.70)
    base.update(kw)
    return S.MovingObstacle(**base)


def test_starts_at_first_waypoint_in_world_frame():
    m = _walker()
    wx, wy, _, _ = world_pose_at(m, 0.0, FLOOR_TOP)
    exp_x, exp_y, _ = S.map_to_world(-5.0, 4.0, 0.0)
    assert (wx, wy) == pytest.approx((exp_x, exp_y))


def test_bottom_sits_on_the_floor():
    """圓柱中心 = 地板 + 高度/2，底部才會貼地。"""
    m = _walker(height=1.70)
    _, _, wz, _ = world_pose_at(m, 0.0, FLOOR_TOP)
    assert wz == pytest.approx(FLOOR_TOP + 0.85)
    assert wz - 0.85 == pytest.approx(FLOOR_TOP)      # 底部貼地


def test_moves_at_the_commanded_speed_in_world_frame():
    """世界座標只是 map 的剛體變換，速率必須守恆。"""
    m = _walker(speed=1.2)
    a = world_pose_at(m, 0.0, FLOOR_TOP)
    b = world_pose_at(m, 1.0, FLOOR_TOP)
    assert math.hypot(b[0] - a[0], b[1] - a[1]) == pytest.approx(1.2)


def test_phase_offset_shifts_the_schedule():
    """phase_s 等效於把時間往前挪。"""
    a = world_pose_at(_walker(phase_s=2.0), 0.0, FLOOR_TOP)
    b = world_pose_at(_walker(phase_s=0.0), 2.0, FLOOR_TOP)
    assert a == pytest.approx(b)


def test_yaw_is_converted_into_world_frame():
    """map→world 有 96.8° 的 yaw 偏移，朝向必須一起轉。"""
    m = _walker(waypoints=((0.0, 0.0), (10.0, 0.0)))
    _, _, _, yaw = world_pose_at(m, 1.0, FLOOR_TOP)
    assert yaw == pytest.approx(0.0 - S.WORLD_TO_MAP_YAW_RAD)


def test_stationary_walker_without_waypoints_does_not_crash():
    m = _walker(waypoints=((-5.0, 4.0),))
    a = world_pose_at(m, 0.0, FLOOR_TOP)
    b = world_pose_at(m, 100.0, FLOOR_TOP)
    assert a == pytest.approx(b)
