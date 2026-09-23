"""對照組成對比較的測試。"""

from __future__ import annotations

import pytest

from compare_arms import pair_runs, paired_delta, sign_test_p


def _run(tag, scen, idx, arrived, minr, coll):
    return {"tag": tag, "scenario": scen, "run_index": idx,
            "arrived": arrived, "legs": 2, "min_range_m": minr,
            "collision_frames": coll}


def test_pair_runs_matches_on_scenario_and_difficulty():
    """★ 配對必須用（情境, 難度）—— 那決定場景長怎樣。
    用 tag 字串配對會因為對照組的 tag 前綴不同而全部配不到。"""
    base = [_run("sa4r2_static_run01", "static", 1, 2, 0.8, 0)]
    arm = [_run("sa4r2_static_run01", "static", 1, 2, 0.6, 3)]
    pairs = pair_runs(base, arm)
    assert len(pairs) == 1
    assert pairs[0][0]["min_range_m"] == 0.8
    assert pairs[0][1]["min_range_m"] == 0.6


def test_pair_runs_drops_unmatched():
    """★ 配不到的不能硬湊 —— 那會拿不同場景相比，結論就是假的。"""
    base = [_run("a", "static", 1, 2, 0.8, 0), _run("b", "mixed", 2, 2, 0.7, 1)]
    arm = [_run("c", "static", 1, 2, 0.6, 3)]
    assert len(pair_runs(base, arm)) == 1


def test_paired_delta_reports_per_pair_and_median():
    base = [_run("a", "static", 1, 2, 0.80, 0), _run("b", "static", 2, 2, 0.60, 2)]
    arm = [_run("c", "static", 1, 2, 0.70, 1), _run("d", "static", 2, 1, 0.50, 5)]
    d = paired_delta(pair_runs(base, arm), "min_range_m")
    assert d["n"] == 2
    assert d["deltas"] == pytest.approx([-0.10, -0.10])
    assert d["median"] == pytest.approx(-0.10)
    assert d["worse"] == 2 and d["better"] == 0


def test_paired_delta_on_no_pairs():
    """★ 沒有配對時要回 n=0，不能回 median=0 —— 那看起來像「沒有差異」。"""
    d = paired_delta([], "min_range_m")
    assert d["n"] == 0
    assert d["median"] is None


def test_sign_test_needs_enough_pairs():
    """★ n 太小就不該報 p 值。12 個成對觀測裡若只有 3 個有差異，
    說「顯著」是過度解讀。"""
    assert sign_test_p(better=6, worse=0) is not None
    assert sign_test_p(better=1, worse=0) is None      # n=1 太少


def test_sign_test_symmetric():
    assert sign_test_p(better=8, worse=0) == pytest.approx(sign_test_p(better=0, worse=8))


def test_sign_test_no_difference_is_not_significant():
    p = sign_test_p(better=4, worse=4)
    assert p is not None and p > 0.5


def test_paired_delta_respects_direction_of_goodness():
    """★ 碰撞幀與耗時是**越小越好**。2026-09-23 第一版少了這個參數，
    差點把「碰撞幀從 0 惡化到 1~25」報成「變好 6 次」。"""
    base = [_run("a", "static", 1, 2, 0.8, 0), _run("b", "static", 2, 2, 0.8, 0)]
    arm = [_run("c", "static", 1, 2, 0.8, 9), _run("d", "static", 2, 2, 0.8, 25)]
    d = paired_delta(pair_runs(base, arm), "collision_frames", higher_is_better=False)
    assert d["worse"] == 2 and d["better"] == 0
    # 若沿用預設（越大越好）就會反過來 —— 這正是當初的錯
    d2 = paired_delta(pair_runs(base, arm), "collision_frames")
    assert d2["better"] == 2
