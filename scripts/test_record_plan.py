"""錄影批次計畫的測試。"""

from __future__ import annotations

import pytest

from record_plan import build_plan, CAMERAS_IN_PLAN, run_tag


def test_thirty_six_clips_from_three_scenarios_four_runs_three_angles():
    """★ 使用者要的是 36 段：3 情境 × 4 趟來回 × 3 視角。"""
    plan = build_plan(runs_per_scenario=4, root="/tmp/rec")
    assert len(plan) == 12                       # 12 趟來回
    assert len(plan) * len(CAMERAS_IN_PLAN) == 36


def test_every_run_has_a_unique_tag():
    """★ 標籤重複 = 後一趟覆蓋前一趟，而且要等全部跑完才發現。"""
    plan = build_plan(runs_per_scenario=4, root="/tmp/rec")
    tags = [r.tag for r in plan]
    assert len(set(tags)) == len(tags)


def test_video_and_bag_share_the_same_tag():
    """★ 使用者要求影片與 rosbag 命名同步，之後才對得起來。"""
    plan = build_plan(runs_per_scenario=2, root="/tmp/rec")
    for r in plan:
        assert r.tag in str(r.bag_dir)
        assert r.tag in str(r.run_dir)
        for cam in CAMERAS_IN_PLAN:
            assert r.tag in str(r.video_path(cam))
            assert cam in r.video_path(cam).name


def test_each_scenario_appears_equally_often():
    plan = build_plan(runs_per_scenario=4, root="/tmp/rec")
    counts: dict[str, int] = {}
    for r in plan:
        counts[r.scenario] = counts.get(r.scenario, 0) + 1
    assert set(counts.values()) == {4}


def test_tag_sorts_in_run_order():
    """★ 用零補位，ls 出來的順序才等於錄影順序（run10 不會排在 run2 前面）。"""
    tags = [run_tag("mixed", i) for i in range(1, 12)]
    assert tags == sorted(tags)


def test_runs_are_numbered_from_one():
    plan = build_plan(runs_per_scenario=2, root="/tmp/rec")
    idx = sorted(r.run_index for r in plan if r.scenario == "mixed")
    assert idx == [1, 2]


def test_rejects_nonpositive_runs():
    with pytest.raises(ValueError):
        build_plan(runs_per_scenario=0, root="/tmp/rec")
