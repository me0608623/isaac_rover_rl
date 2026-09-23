"""scene.json 的測試。"""

from __future__ import annotations

import pytest

from ros_graph_spec import Obstacle
from scene_snapshot import (build_snapshot, obstacles_of, read_snapshot,
                            still_bodies, write_snapshot)


def _snap():
    obs = [Obstacle("prop_4_0", -5.7, 4.3, "prop", height=2.22, size_x=0.419,
                    size_y=1.84, yaw_deg=-80.0, asset="SM_Cupboard"),
           Obstacle("pair_4_1", -12.5, 5.0, "person")]
    chars = [("Character_17", -12.5, 5.0, False), ("Character_10", -2.0, 5.5, True)]
    return build_snapshot("mixed", 4, obs, chars)


def test_roundtrip_keeps_every_obstacle_field(tmp_path):
    """★ 道具的 asset / yaw / bbox 缺一個，分析就會把道具擺錯方向或大小。"""
    write_snapshot(tmp_path, _snap())
    back = obstacles_of(read_snapshot(tmp_path))
    assert back[0] == Obstacle("prop_4_0", -5.7, 4.3, "prop", height=2.22,
                               size_x=0.419, size_y=1.84, yaw_deg=-80.0,
                               asset="SM_Cupboard")
    assert back[1].kind == "person"


def test_still_bodies_excludes_walkers(tmp_path):
    write_snapshot(tmp_path, _snap())
    assert still_bodies(read_snapshot(tmp_path)) == [(-12.5, 5.0)]


def test_missing_snapshot_returns_none(tmp_path):
    """★ 舊錄影沒有 scene.json —— 回 None 讓呼叫端決定，不可假裝有。"""
    assert read_snapshot(tmp_path) is None


def test_wrong_version_is_rejected(tmp_path):
    s = _snap()
    s["version"] = 99
    write_snapshot(tmp_path, s)
    with pytest.raises(ValueError):
        read_snapshot(tmp_path)


def test_v2_roundtrips_walks_and_standing_yaw(tmp_path):
    """★★ 回放要能只靠快照重建場景：走動行人的路線與站立人物的朝向都要在。

    2026-09-23 兩遍各寫一份規則已經走樣 —— dynamic 與 static 的影片裡會出現
    導航時根本不在場的人。回放改成照抄快照，快照就必須完整。
    """
    from ros_graph_spec import CharacterWalk
    from scene_snapshot import placed_standing, walks_of

    walk = CharacterWalk("Character_10", ((-2.0, 5.5), (-12.0, 4.4)), speed=0.85,
                         phase_s=3.2)
    snap = build_snapshot(
        "mixed", 4,
        [Obstacle("pair_c27_4_1", -12.5, 5.0, "person")],
        [("Character_17", -12.5, 5.0, False), ("Character_10", -2.0, 5.5, True),
         ("Character_11", -10.0, 4.6, False)],                # 停在原位、沒擺位
        walks=[walk], standing_yaw={"Character_17": -1.4}, route="c27")
    write_snapshot(tmp_path, snap)
    back = read_snapshot(tmp_path)
    assert back["route"] == "c27"
    assert walks_of(back) == [walk]
    assert placed_standing(back) == [("Character_17", -12.5, 5.0, -1.4)]
    # 停在原位的人不算「擺位」，但仍是靜止的身體
    assert (-10.0, 4.6) in still_bodies(back)


def test_v1_snapshot_is_readable_for_analysis_but_not_for_replay(tmp_path):
    """★ 舊的 v1 快照還能拿來分析（障礙、角色位置都在），但缺路線資訊，
    拿去回放要大聲失敗，不可猜。"""
    import json

    from scene_snapshot import walks_of

    (tmp_path / "scene.json").write_text(json.dumps(
        {"version": 1, "scenario": "mixed", "run_index": 4,
         "obstacles": [], "characters": []}))
    snap = read_snapshot(tmp_path)
    assert snap["version"] == 1
    with pytest.raises(ValueError):
        walks_of(snap)


def test_replay_copies_the_snapshot_instead_of_recomputing_rules():
    """★★ 2026-09-23：第一遍與回放各寫一份「誰該停用」的規則，改了一邊忘了
    另一邊，dynamic/static 的影片裡出現導航時根本不在場的人。
    回放必須照抄快照，不可以自己算 variant() 或套停用規則。"""
    import pathlib

    src = (pathlib.Path(__file__).parent / "replay_render.py").read_text()
    code = "\n".join(l for l in src.splitlines() if not l.strip().startswith("#"))
    for banned in ("variant(", "thin_by_spacing(", "too_close_to_routing_node(",
                   "too_close_to_robot("):
        assert banned not in code, f"replay_render 又在自己算 {banned}"
    assert "read_snapshot(" in code and "--scene" in code
