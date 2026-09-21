"""角色碰撞體的安全距離測試。"""

from __future__ import annotations

import pytest

from character_colliders import MIN_CLEARANCE_FROM_ROBOT_M, too_close_to_robot

ROBOT_SPAWN_WORLD = (0.166, 2.943)     # build_ros_graph 實際算出的出生點


def test_character_overlapping_spawn_is_rejected():
    """★ 實際炸過：Character_09 在 world(0.00, 2.62)，距出生點 0.36 m，
    kinematic 碰撞體把 base_footprint 彈到 (-2048, -256, -256)。"""
    assert too_close_to_robot((0.00, 2.62), ROBOT_SPAWN_WORLD) is True


def test_distant_character_is_allowed():
    """Character_10 在 world(0.00, 10.00)，距出生點 7 m，安全。"""
    assert too_close_to_robot((0.00, 10.00), ROBOT_SPAWN_WORLD) is False


def test_threshold_covers_robot_plus_person_radius():
    """門檻至少要大於 車體半徑0.35 + 人體半徑0.25 = 0.60 m，留餘裕。"""
    assert MIN_CLEARANCE_FROM_ROBOT_M > 0.35 + 0.25


def test_boundary_is_exclusive():
    r = MIN_CLEARANCE_FROM_ROBOT_M
    assert too_close_to_robot((ROBOT_SPAWN_WORLD[0] + r, ROBOT_SPAWN_WORLD[1]),
                              ROBOT_SPAWN_WORLD) is False
    assert too_close_to_robot((ROBOT_SPAWN_WORLD[0] + r - 1e-6, ROBOT_SPAWN_WORLD[1]),
                              ROBOT_SPAWN_WORLD) is True


# ---------------------------------------------- 與機器人的碰撞過濾
from character_colliders import ROBOT_ARTICULATION_PATH


def test_robot_path_points_at_the_articulation_root():
    """過濾要掛在 articulation root 上，才能一次涵蓋所有 link。"""
    assert ROBOT_ARTICULATION_PATH.endswith("charger_rover_urdf5")
    assert ROBOT_ARTICULATION_PATH.startswith("/World/")
