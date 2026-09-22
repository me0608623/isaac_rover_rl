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


# ── routing 點位淨空 ────────────────────────────────────────────────────
STATIONS = {"c28": (-0.06, 5.95, 0.0), "c25": (-14.22, 5.38, 0.0),
            "c24": (3.24, 6.28, 0.0), "c26": (-10.73, 4.51, 0.0)}


def test_nearest_routing_node_finds_the_closest():
    from character_colliders import nearest_routing_node

    name, d = nearest_routing_node((-0.06, 5.95), STATIONS)
    assert name == "c28"
    assert d == pytest.approx(0.0, abs=1e-9)


def test_standing_person_on_a_routing_node_is_flagged():
    """★ 站著的人立在導航點位上會擋住目標 —— 2026-09-22 實測
    Character_15 離 c24 只有 0.40 m、Character_09 離 c28 只有 0.36 m。"""
    from character_colliders import too_close_to_routing_node

    assert too_close_to_routing_node((3.24 + 0.40, 6.28), STATIONS) is True
    assert too_close_to_routing_node((-0.06, 5.95 + 0.36), STATIONS) is True


def test_person_well_clear_of_every_node_is_not_flagged():
    from character_colliders import too_close_to_routing_node

    assert too_close_to_routing_node((-5.0, 1.0), STATIONS) is False


def test_routing_clearance_boundary_is_one_metre():
    from character_colliders import (MIN_CLEARANCE_FROM_ROUTING_NODE_M,
                                     too_close_to_routing_node)

    assert MIN_CLEARANCE_FROM_ROUTING_NODE_M == 1.0
    assert too_close_to_routing_node((-0.06, 5.95 + 0.99), STATIONS) is True
    assert too_close_to_routing_node((-0.06, 5.95 + 1.01), STATIONS) is False


def test_empty_station_table_never_flags():
    """★ 讀不到站點表時要當成「沒有限制」，不能把全部角色都停用 ——
    那會安靜地錄出一條沒有半個人的走廊。"""
    from character_colliders import nearest_routing_node, too_close_to_routing_node

    assert too_close_to_routing_node((0.0, 0.0), {}) is False
    assert nearest_routing_node((0.0, 0.0), {}) == (None, float("inf"))


# ── 站立行人的疏密 ──────────────────────────────────────────────────────
def test_thin_by_spacing_drops_the_crowded_ones():
    """★ 使用者 2026-09-22 指定「減少行人在一起的密度」。
    兩個人站在 0.5 m 內只會擠成一團，留一個就好。"""
    from character_colliders import thin_by_spacing

    kept, dropped = thin_by_spacing(
        [("a", (0.0, 0.0)), ("b", (0.4, 0.0)), ("c", (10.0, 0.0))], 2.5)
    assert kept == ["a", "c"]
    assert dropped == ["b"]


def test_thin_by_spacing_keeps_everyone_when_already_spread():
    from character_colliders import thin_by_spacing

    kept, dropped = thin_by_spacing(
        [("a", (0.0, 0.0)), ("b", (5.0, 0.0)), ("c", (10.0, 0.0))], 2.5)
    assert len(kept) == 3 and dropped == []


def test_thin_by_spacing_is_deterministic_regardless_of_input_order():
    """★ 第一遍（導航）與第二遍（回放算圖）必須留下**同一批人**，
    否則影片裡的人數與 rosbag 對不起來。順序不同就留下不同的人 = 不可用。"""
    from character_colliders import thin_by_spacing

    items = [("c", (10.0, 0.0)), ("a", (0.0, 0.0)), ("b", (0.4, 0.0))]
    assert thin_by_spacing(items, 2.5)[0] == thin_by_spacing(sorted(items), 2.5)[0]


def test_thin_by_spacing_with_zero_spacing_keeps_all():
    from character_colliders import thin_by_spacing

    items = [("a", (0.0, 0.0)), ("b", (0.0, 0.0))]
    assert len(thin_by_spacing(items, 0.0)[0]) == 2
