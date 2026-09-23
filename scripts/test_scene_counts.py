"""場上靜態/動態數量的測試。用真的 log 字串。"""

from __future__ import annotations

from scene_counts import counts, counts_from_log, parse_log

_STATIC = ("[run_isaac_sim] 情境 static／變體 run4：靜態障礙 6/18 啟用　行人走動 關\n"
           "[run_isaac_sim] 程序化步態：13 人 / 104 個擺動關節 / 0 人沿路徑移動"
           "　站立人物擺位 5 個　腳底對地 13 個\n")
_DYNAMIC = ("[run_isaac_sim] 情境 dynamic／變體 run4：靜態障礙 0/18 啟用　行人走動 開\n"
            "[run_isaac_sim] 程序化步態：13 人 / 104 個擺動關節 / 8 人沿路徑移動"
            "　站立人物擺位 5 個　腳底對地 5 個\n")
_MIXED = ("[run_isaac_sim] 情境 mixed／變體 run4：靜態障礙 6/18 啟用　行人走動 開\n"
          "[run_isaac_sim] 程序化步態：13 人 / 104 個擺動關節 / 8 人沿路徑移動"
          "　站立人物擺位 5 個　腳底對地 5 個\n")


def test_parses_the_real_log_lines():
    i = parse_log(_MIXED)
    assert i == {"scenario": "mixed", "run_index": 4, "obstacles_enabled": 6,
                 "chars": 13, "walking": 8, "standing": 5}


def test_static_counts_the_parked_walkers_as_static():
    """★★ static 情境的走動人物**沒有消失**，是停在 USD 原位不動 ——
    把它們漏掉會讓 static 看起來比 mixed 輕鬆（實際上最難）。
    6 個障礙 + 13 個不動的角色 − 5 個疊在障礙上的 = 14 靜態。"""
    assert counts_from_log(_STATIC) == (14, 0)


def test_dynamic_still_has_the_standing_people():
    """★★ dynamic 印「靜態障礙 0/18 啟用」只關圓柱，
    站立人物仍在場、光達打得到 —— 靜態不是 0 而是 5。"""
    assert counts_from_log(_DYNAMIC) == (5, 8)


def test_mixed_does_not_double_count_the_overlap():
    """★ 站立人物擺在 person 障礙圓柱上，不扣重疊會從 6 變 11。"""
    assert counts_from_log(_MIXED) == (6, 8)


def test_overlap_is_not_subtracted_when_obstacles_are_off():
    """★ 障礙關閉時沒有可重疊的圓柱，扣掉會把站立人物憑空消掉。"""
    assert counts({"obstacles_enabled": 0, "chars": 13, "walking": 8,
                   "standing": 5}) == (5, 8)


def test_returns_none_when_the_lines_are_missing():
    """★ 湊預設值會讓檔名上的假數字看起來一樣可信。"""
    assert counts_from_log("") is None
    assert counts_from_log("[run_isaac_sim] 情境 mixed／變體 run4：靜態障礙 6/18 啟用") is None
    assert parse_log("random text") is None


def test_every_recorded_run_is_parseable():
    """★ 36 趟主批次 + 36 趟對照全部要讀得出來，否則索引會出現「數量不明」。"""
    import pathlib
    ws = pathlib.Path(__file__).resolve().parent.parent
    logs = (list(ws.glob("recordings/模型*/*/isaac_nav.log"))
            + list(ws.glob("recordings_abl/*/*/isaac_nav.log")))
    if not logs:
        import pytest
        pytest.skip("這台沒有錄影產物")
    bad = [p for p in logs if counts_from_log(p.read_text(errors="replace")) is None]
    assert not bad, f"讀不出場景數量：{[str(p) for p in bad[:5]]}"
