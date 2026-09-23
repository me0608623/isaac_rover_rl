"""「最近的是誰」分解的測試。"""

from __future__ import annotations

import math

import pytest

from nearest_source import (CATEGORIES, PED_RADIUS_M, STANDING_RADIUS_M,
                            align_offset, box_surface_distance, classify,
                            cylinder_surface_distance)


def test_cylinder_surface_distance():
    assert cylinder_surface_distance((0.0, 0.0), (3.0, 4.0), 0.25) == pytest.approx(4.75)


def test_cylinder_distance_clamps_at_zero_inside():
    """★ 車跑到障礙裡面時距離要夾在 0，不能變負 —— 負值會讓
    argmin 永遠選中它，把所有幀都歸給那一個障礙。"""
    assert cylinder_surface_distance((0.0, 0.0), (0.1, 0.0), 0.25) == 0.0


def test_box_surface_distance_axis_aligned():
    assert box_surface_distance((1.35, 0.0), (0.0, 0.0), 0.35, 0.25) == pytest.approx(1.0)


def test_box_surface_distance_diagonal():
    d = box_surface_distance((0.35 + 3.0, 0.25 + 4.0), (0.0, 0.0), 0.35, 0.25)
    assert d == pytest.approx(5.0)


def test_box_distance_zero_when_inside():
    assert box_surface_distance((0.1, 0.1), (0.0, 0.0), 0.35, 0.25) == 0.0


def test_classify_picks_the_closest_source():
    assert classify({"走動行人": 0.5, "牆": 2.0}) == "走動行人"
    assert classify({"走動行人": 1.5, "箱型障礙": 0.4, "牆": 2.0}) == "箱型障礙"
    assert classify({"走動行人": 1.5, "牆": 0.3}) == "牆"


def test_classify_ignores_missing_sources():
    """★ static 沒有走動行人、dynamic 沒有箱型障礙 —— 缺項是常態，
    不能因為有一個 None 就整組壞掉或被 None 贏走。"""
    assert classify({"走動行人": None, "箱型障礙": 0.4, "牆": 2.0}) == "箱型障礙"
    assert classify({"走動行人": None, "箱型障礙": None, "牆": 0.3}) == "牆"


def test_classify_with_nothing_returns_none():
    assert classify({"走動行人": None, "牆": None}) is None


def test_categories_cover_every_physical_source():
    """★ 少一類就會被歸到「牆」，那正是這支程式要查清楚的事。"""
    assert set(CATEGORIES) == {"走動行人", "站立行人", "箱型障礙", "牆"}


def test_radii_are_documented_and_sane():
    assert 0.15 <= PED_RADIUS_M <= 0.30
    assert PED_RADIUS_M <= STANDING_RADIUS_M, "站立人物腳邊多一根圓柱，半徑不會更小"


class _S:
    def __init__(self, t, pos):
        self.t = t
        self.pos = pos
        self.quat = (1.0, 0.0, 0.0, 0.0)


def _straight_line_poses(n=200, dt=0.05):
    """沿 world +x 等速前進的假軌跡。"""
    return [_S(i * dt, (i * dt * 0.5, 0.0, 0.0)) for i in range(1, n + 1)]


def test_align_offset_recovers_a_known_shift():
    """★ nav CSV 的時間從「該段開始」算，pose.csv 從模擬開始算；
    對不上的話行人位置會查到錯的時刻，整個分解就沒有意義。"""
    import ros_graph_spec as S

    from pose_log import pose_at

    poses = _straight_line_poses()
    true_off = 2.0
    nav = []
    for i in range(0, 70):                       # 最後一筆 6.9+2.0 s，仍在軌跡內
        t = i * 0.1
        s = pose_at(poses, t + true_off)
        mx, my, _ = S.world_to_map(s.pos[0], s.pos[1], 0.0)
        nav.append((t, mx, my))
    off, rms = align_offset(nav, poses)
    assert off == pytest.approx(true_off, abs=0.1)
    assert rms < 0.1


def test_align_offset_reports_inf_when_there_is_nothing_to_align():
    """★ 空資料要回 inf 讓呼叫端看得出「沒對上」，不能安靜地回 0。"""
    assert align_offset([], _straight_line_poses())[1] == math.inf
    assert align_offset([(0.0, 0.0, 0.0)], [])[1] == math.inf


def test_read_pgm_handles_a_comment_right_after_the_magic(tmp_path):
    """★ 這份佔據圖是 GIMP 存的，magic 之後第一行就是註解。
    只用 readline() 讀三行會把註解當成寬高，整張圖位移。"""
    import numpy as np

    from nearest_source import read_pgm

    f = tmp_path / "t.pgm"
    f.write_bytes(b"P5\n# Created by GIMP\n3 2\n255\n" + bytes([0, 1, 2, 3, 4, 5]))
    a = read_pgm(f)
    assert a.shape == (2, 3)
    assert np.array_equal(a, np.array([[0, 1, 2], [3, 4, 5]], dtype=np.uint8))


def test_read_pgm_rejects_ascii_pgm(tmp_path):
    """★ P2（ASCII）用二進位的方式讀會得到一張雜訊圖，要大聲拒絕。"""
    from nearest_source import read_pgm

    f = tmp_path / "t.pgm"
    f.write_bytes(b"P2\n2 1\n255\n0 255\n")
    with pytest.raises(ValueError):
        read_pgm(f)


def test_read_pgm_matches_the_real_occupancy_map():
    """★ 真圖的長寬要跟標頭一致（1747×1935），否則距離場整個錯位。"""
    import pathlib

    from nearest_source import read_pgm

    p = pathlib.Path(__file__).resolve().parent.parent / "map" / "4v3F.pgm"
    if not p.exists():
        pytest.skip("沒有佔據圖")
    assert read_pgm(p).shape == (1935, 1747)


def test_unexplained_slack_is_big_enough_to_be_a_real_gap():
    """★ 這個門檻太小會把正常的量測誤差（光達離散化 + NDT 0.09 m）
    誤判成「來源不明」；太大則會把真的對不上的幀硬塞給最近的一類。"""
    from nearest_source import NEAR_THRESHOLD_M, UNEXPLAINED_SLACK_M

    assert 0.3 <= UNEXPLAINED_SLACK_M <= 1.0
    assert UNEXPLAINED_SLACK_M > NEAR_THRESHOLD_M / 2
