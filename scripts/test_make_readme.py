"""結果 README 產生器的測試。"""

from __future__ import annotations

import pytest

from make_readme import cell_summary, parse_nav_table, render


NAV = """
── 第 1 段：c28 → c25 @ map(-14.22,+5.38) ──
  抵達  耗時 31.5 s  路徑 20.76 m  距目標 1.00 m
  最近障礙 最小 0.84 m / 中位 1.21 m　碰撞幀 0/314　|v| 平均 0.634
── 第 2 段：c25 → c28 @ map(-0.06,+5.95) ──
  抵達  耗時 38.4 s  路徑 22.88 m  距目標 0.97 m
  最近障礙 最小 0.77 m / 中位 1.05 m　碰撞幀 2/384　|v| 平均 0.551
==============================================================
段            結果          耗時      路徑     最近障礙      碰撞幀
c28→c25      OK       31.5s  20.76m    0.84m    0/314
c25→c28      OK       38.4s  22.88m    0.77m    2/384
"""


def test_parse_nav_table_reads_both_legs():
    legs = parse_nav_table(NAV)
    assert len(legs) == 2
    assert legs[0]["result"] == "OK"
    assert legs[0]["seconds"] == pytest.approx(31.5)
    assert legs[0]["path_m"] == pytest.approx(20.76)
    assert legs[0]["min_range_m"] == pytest.approx(0.84)
    assert legs[0]["collision_frames"] == 0
    assert legs[1]["collision_frames"] == 2


def test_parse_nav_table_reads_failures():
    legs = parse_nav_table("c28→c25      FAIL    200.0s  30.26m    0.47m    0/1425\n")
    assert legs[0]["result"] == "FAIL"
    assert legs[0]["seconds"] == pytest.approx(200.0)


def test_parse_nav_table_on_garbage_returns_nothing():
    """★ 解析失敗要回空，不能回一筆看起來正常的零 —— 那會讓失敗的趟
    在彙總表裡長得像成功。"""
    assert parse_nav_table("完全不是表格") == []


def test_cell_summary_aggregates():
    legs = [{"result": "OK", "seconds": 10.0, "min_range_m": 0.8, "collision_frames": 0},
            {"result": "OK", "seconds": 20.0, "min_range_m": 0.6, "collision_frames": 3},
            {"result": "FAIL", "seconds": 200.0, "min_range_m": 0.4, "collision_frames": 1}]
    c = cell_summary(legs)
    assert c["legs"] == 3
    assert c["arrived"] == 2
    assert c["min_range_m"] == pytest.approx(0.4)     # 取最差
    assert c["collision_frames"] == 4                 # 加總


def test_cell_summary_on_empty():
    c = cell_summary([])
    assert c["legs"] == 0 and c["arrived"] == 0


def test_render_includes_every_run():
    runs = [{"tag": "sa4r2_static_run01", "model": "sa4r2", "scenario": "static",
             "run_index": 1, "speed_rate": 0.7, "videos": ["a.mp4"],
             "legs": [{"result": "OK", "seconds": 10.0, "min_range_m": 0.8,
                       "collision_frames": 0}]}]
    text = render(runs)
    assert "sa4r2_static_run01" in text
    assert "static" in text


def test_density_table_comes_from_the_runs_not_a_hardcoded_copy():
    """★★ 靜/動數量要從各趟 log 算。2026-09-23 README 裡這張表原本是手抄的
    舊批次數字，重錄（新路線、分層抽樣、道具）之後就全錯了，而且看不出來。"""
    from make_readme import density_table

    runs = [
        {"scenario": "static", "run_index": 1, "counts": (4, 0)},
        {"scenario": "dynamic", "run_index": 1, "counts": (0, 2)},
        {"scenario": "mixed", "run_index": 1, "counts": (3, 2)},
        {"scenario": "mixed", "run_index": 2, "counts": None},
    ]
    t = "\n".join(density_table(runs))
    assert "靜4動0" in t and "靜0動2" in t and "靜3動2" in t
    assert "—" in t                          # 讀不到的格子標「—」，不填 0


def test_density_table_shows_disagreeing_models_instead_of_picking_one():
    """★ 同一格不同模型數字不一樣，代表執行期停用的角色不同 —— 全部列出。"""
    from make_readme import density_table

    runs = [{"scenario": "mixed", "run_index": 4, "counts": (6, 8)},
            {"scenario": "mixed", "run_index": 4, "counts": (6, 7)}]
    t = "\n".join(density_table(runs))
    assert "靜6動7 / 靜6動8" in t
