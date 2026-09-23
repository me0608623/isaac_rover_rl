"""錄影產物健檢的測試。"""

from __future__ import annotations

import pytest

from check_recordings import (expected_frames, frame_count_ok, is_too_dark,
                              summarise)


def test_expected_frames_from_duration_and_fps():
    assert expected_frames(10.0, 30) == 300


def test_frame_count_ok_allows_small_slack():
    """★ BasicWriter 收尾時常少幾幀（實測 2582 幀只寫出 2578），
    差幾幀是正常的，差一大截才是壞掉。"""
    assert frame_count_ok(2578, 2582) is True
    assert frame_count_ok(2500, 2582) is False


def test_frame_count_ok_rejects_zero():
    """★ 0 幀一定是壞的 —— 先前算圖全黑那次就是寫出 0 幀。"""
    assert frame_count_ok(0, 100) is False


def test_black_detection():
    """★ 全黑是這個專案踩過最久的坑（RTX Real-Time 算不出光照）。
    健檢一定要抓得到。"""
    assert is_too_dark(mean_luma=0.0) is True
    assert is_too_dark(mean_luma=1.2) is True       # 只有背景天空
    assert is_too_dark(mean_luma=140.0) is False


def test_summarise_counts_problems():
    rows = [
        {"tag": "a", "camera": "chase", "frames": 100, "expected": 100, "mean": 140.0},
        {"tag": "b", "camera": "chase", "frames": 0, "expected": 100, "mean": 0.0},
        {"tag": "c", "camera": "chase", "frames": 100, "expected": 100, "mean": 0.5},
    ]
    s = summarise(rows)
    assert s["total"] == 3
    assert s["ok"] == 1
    assert sorted(s["bad_tags"]) == ["b", "c"]


def test_summarise_on_empty_is_not_a_pass():
    """★ 沒有任何影片時不能回報「全部正常」—— 那會讓人以為跑完了。"""
    s = summarise([])
    assert s["total"] == 0
    assert s["ok"] == 0
    assert s["all_ok"] is False


def test_unmeasured_is_not_reported_as_black():
    """★ 2026-09-23 踩到：亮度解析失敗回 0.0，害 108 段全部被判全黑，
    而實際亮度是 111~206。量不到必須與「全黑」分開，否則失敗的量測會
    偽裝成壞結果 —— 那比沒量更糟。"""
    from check_recordings import is_too_dark, is_unmeasured

    assert is_too_dark(None) is False
    assert is_unmeasured(None) is True
    assert is_unmeasured(0.0) is False


def test_summarise_separates_unmeasured_from_bad():
    rows = [{"tag": "a", "camera": "chase", "frames": 100, "expected": 100, "mean": None},
            {"tag": "b", "camera": "chase", "frames": 100, "expected": 100, "mean": 0.0},
            {"tag": "c", "camera": "chase", "frames": 100, "expected": 100, "mean": 150.0}]
    s = summarise(rows)
    assert s["bad_tags"] == ["b"]
    assert s["unmeasured_tags"] == ["a"]
    assert s["ok"] == 1
