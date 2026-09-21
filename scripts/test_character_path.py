"""角色沿路徑行走的測試。"""

from __future__ import annotations

import math

import pytest

import ros_graph_spec as S
from character_path import character_pose_at, facing_rotation_deg

FLOOR = -0.3233


def test_rest_facing_is_minus_y():
    """★ 由解剖實測：腳趾指向 -Y（L_Foot→L_ToeBase 方向 (+0.03,-0.96,-0.27)），
    左肩在 +X。弄反的話角色會倒著走。"""
    assert S.CHARACTER_REST_FACING_DEG == pytest.approx(-90.0)


def test_facing_north_needs_ninety_degrees():
    """朝 +Y（yaw=90°）時，需從 -Y 轉 180°。"""
    assert facing_rotation_deg(math.pi / 2) == pytest.approx(180.0)


def test_facing_rest_direction_needs_no_rotation():
    assert facing_rotation_deg(math.radians(-90.0)) == pytest.approx(0.0)


def test_facing_plus_x_turns_ninety():
    assert facing_rotation_deg(0.0) == pytest.approx(90.0)


def test_character_feet_stay_on_the_floor():
    """★ 角色的原點在腳底，所以 z 就是地板高度 —— 不能像圓柱那樣加半個身高。"""
    w = S.CharacterWalk("X", waypoints=((-7.0, 4.5), (-3.0, 4.5)), speed=1.0)
    _, _, wz, _ = character_pose_at(w, 0.0, FLOOR)
    assert wz == pytest.approx(FLOOR)


def test_character_moves_at_the_commanded_speed():
    w = S.CharacterWalk("X", waypoints=((-7.0, 4.5), (-3.0, 4.5)), speed=1.1)
    a = character_pose_at(w, 0.0, FLOOR)
    b = character_pose_at(w, 1.0, FLOOR)
    assert math.hypot(b[0] - a[0], b[1] - a[1]) == pytest.approx(1.1)


def test_standing_character_has_no_waypoints():
    """沒有路徑的角色站在原地 —— 但仍要套基礎姿勢，不能退回 T-pose。"""
    w = S.CharacterWalk("X", waypoints=(), speed=0.0)
    a = character_pose_at(w, 0.0, FLOOR)
    b = character_pose_at(w, 50.0, FLOOR)
    assert a == pytest.approx(b)


def test_heading_follows_the_path_direction():
    """沿 +x 走時，世界朝向應指向該段路徑的方向。"""
    w = S.CharacterWalk("X", waypoints=((-7.0, 4.5), (-3.0, 4.5)), speed=1.0)
    _, _, _, yaw = character_pose_at(w, 1.0, FLOOR)
    # map frame 朝 +x（yaw=0）→ world yaw = 0 - WORLD_TO_MAP_YAW_RAD
    assert yaw == pytest.approx(-S.WORLD_TO_MAP_YAW_RAD)


def test_all_default_walks_stay_inside_the_corridor():
    """★ 路徑點跑到牆外，角色會穿牆走。走廊約 map x∈[-22,2], y∈[3,8]。"""
    for w in S.DEFAULT_CHARACTER_WALKS:
        for x, y in w.waypoints:
            assert -22.0 <= x <= 2.0, f"{w.name} 路徑點 x={x} 超出走廊"
            assert 3.0 <= y <= 8.0, f"{w.name} 路徑點 y={y} 超出走廊"


def test_default_walks_clear_the_static_obstacles():
    """★ 路徑不可穿過靜態障礙物 —— 兩個碰撞體疊在一起既不真實也會干擾量測。"""
    for w in S.DEFAULT_CHARACTER_WALKS:
        for x, y in w.waypoints:
            for o in S.DEFAULT_OBSTACLES:
                d = math.hypot(x - o.map_x, y - o.map_y)
                assert d > 0.8, f"{w.name} 路徑點 ({x},{y}) 距 {o.name} 僅 {d:.2f} m"


def test_default_walk_speeds_are_human():
    """成人步行 0.6~1.6 m/s。"""
    for w in S.DEFAULT_CHARACTER_WALKS:
        assert 0.5 <= w.speed <= 1.6, f"{w.name} 速度 {w.speed} 不像人"
