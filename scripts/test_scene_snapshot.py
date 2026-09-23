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
