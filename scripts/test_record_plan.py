"""錄影批次計畫的測試。"""

from __future__ import annotations

import pytest

from record_plan import build_plan, CAMERAS_IN_PLAN, DEFAULT_MODELS, run_tag


def test_plan_covers_every_model_scenario_and_run():
    """★ 3 模型 × 3 情境 × 4 趟 × 3 視角 = 108 段。

    模型清單是 deploy_select.sh 選單扣掉第一項（那顆在 sim 端沒有 profile）。
    """
    assert DEFAULT_MODELS == ("sa4r2", "sa4r3", "sa5r2")
    plan = build_plan(runs_per_scenario=4, root="/tmp/rec")
    assert len(plan) == 36                       # 36 趟來回
    assert len(plan) * len(CAMERAS_IN_PLAN) == 108


def test_each_model_gets_the_full_scenario_set():
    plan = build_plan(runs_per_scenario=4, root="/tmp/rec")
    for m in DEFAULT_MODELS:
        rows = [r for r in plan if r.model == m]
        assert len(rows) == 12
        assert {r.scenario for r in rows} == {"static", "dynamic", "mixed"}


def test_one_model_finishes_before_the_next_starts():
    """★ 模型放最外層：中途停手至少會有**完整的一個模型**可用，
    而不是三個模型各缺一半。"""
    plan = build_plan(runs_per_scenario=2, root="/tmp/rec")
    order = [r.model for r in plan]
    assert order == sorted(order, key=lambda m: DEFAULT_MODELS.index(m))


def test_empty_model_list_is_rejected():
    with pytest.raises(ValueError):
        build_plan(runs_per_scenario=1, root="/tmp/rec", models=())


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
    assert set(counts.values()) == {4 * len(DEFAULT_MODELS)}


def test_tag_carries_the_model_so_runs_never_collide():
    """★ 少了模型維度，三個模型會寫進同一個目錄互相覆蓋，
    而且要等全部跑完才發現。"""
    plan = build_plan(runs_per_scenario=1, root="/tmp/rec")
    assert len({r.tag for r in plan}) == len(plan)
    for r in plan:
        assert r.model in r.tag


def test_tag_sorts_in_run_order():
    """★ 用零補位，ls 出來的順序才等於錄影順序（run10 不會排在 run2 前面）。"""
    tags = [run_tag("sa4r2", "mixed", i) for i in range(1, 12)]
    assert tags == sorted(tags)


def test_runs_are_numbered_from_one():
    plan = build_plan(runs_per_scenario=2, root="/tmp/rec")
    idx = sorted(r.run_index for r in plan
                 if r.scenario == "mixed" and r.model == "sa4r2")
    assert idx == [1, 2]


def test_rejects_nonpositive_runs():
    with pytest.raises(ValueError):
        build_plan(runs_per_scenario=0, root="/tmp/rec")
