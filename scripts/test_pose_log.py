"""位姿軌跡記錄 / 回放的測試。"""

from __future__ import annotations

import math

import pytest

from pose_log import PoseSample, format_row, parse_rows, pose_at


def _s(t, x=0.0, y=0.0, z=0.0, q=(1.0, 0.0, 0.0, 0.0)):
    return PoseSample(t, (x, y, z), q)


def test_row_roundtrips():
    s = _s(1.25, 2.0, -3.0, 0.5, (0.7071068, 0.0, 0.0, 0.7071068))
    back = parse_rows([format_row(s)])[0]
    assert back.t == pytest.approx(s.t)
    assert back.pos == pytest.approx(s.pos, abs=1e-6)
    assert back.quat == pytest.approx(s.quat, abs=1e-6)


def test_header_lines_are_ignored():
    assert parse_rows(["t,x,y,z,qw,qx,qy,qz", "0,0,0,0,1,0,0,0"])[0].t == 0.0


def test_pose_at_interpolates_position():
    samples = [_s(0.0, 0.0), _s(1.0, 10.0)]
    assert pose_at(samples, 0.25).pos[0] == pytest.approx(2.5)


def test_pose_at_interpolates_rotation_the_short_way():
    """★ 四元數插值要走短弧。直接線性內插相反號的四元數會轉一大圈，
    回放出來的車會在原地甩頭。"""
    a = _s(0.0, q=(1.0, 0.0, 0.0, 0.0))
    b = _s(1.0, q=(-0.9999619, 0.0, 0.0, -0.0087265))   # 幾乎同一個姿態，但號相反
    mid = pose_at([a, b], 0.5)
    ang = 2 * math.degrees(math.acos(min(1.0, abs(mid.quat[0]))))
    assert ang < 1.0, f"插出來轉了 {ang:.1f}°，走了長弧"


def test_pose_at_clamps_outside_the_range():
    """★ 超出範圍要夾住，不能外插 —— 外插會讓車在影片結尾飛出去。"""
    samples = [_s(1.0, 5.0), _s(2.0, 6.0)]
    assert pose_at(samples, 0.0).pos[0] == pytest.approx(5.0)
    assert pose_at(samples, 9.0).pos[0] == pytest.approx(6.0)


def test_pose_at_is_exact_on_samples():
    samples = [_s(0.0, 1.0), _s(1.0, 2.0), _s(2.0, 4.0)]
    assert pose_at(samples, 1.0).pos[0] == pytest.approx(2.0)


def test_empty_log_is_rejected_loudly():
    """★ 空的軌跡檔要報錯，不能安靜地回傳原點 —— 那會錄出一整段車不動的影片。"""
    with pytest.raises(ValueError):
        pose_at([], 0.0)


def test_parse_skips_blank_and_bad_rows():
    rows = ["t,x,y,z,qw,qx,qy,qz", "", "0,0,0,0,1,0,0,0", "壞掉的一行", "1,1,0,0,1,0,0,0"]
    assert len(parse_rows(rows)) == 2


def test_motion_window_trims_the_idle_head_and_tail():
    """★ 第一趟的 Isaac 比 ROS 早開約一分鐘，導航結束後也還會多跑幾秒。
    不裁掉的話，每段影片開頭都有一大段車不動的畫面，算圖時間也白花。"""
    from pose_log import motion_window

    s = ([_s(t / 30.0, 0.0) for t in range(0, 60)]        # 前 2 s 不動
         + [_s(2.0 + i / 30.0, i * 0.05) for i in range(1, 61)]   # 動 2 s
         + [_s(4.0 + t / 30.0, 3.0) for t in range(0, 90)])       # 後 3 s 不動
    t0, t1 = motion_window(s, lead=0.5, tail=1.0)
    # 要求的是「涵蓋整段運動、又不要把靜止的頭尾整段收進來」，
    # 不是某個精確的秒數（中央差分視窗本來就會有一點提前/延後）。
    assert t0 <= 2.0, f"t0={t0:.2f} 已經切掉運動的開頭"
    assert t0 >= 1.0, f"t0={t0:.2f} 幾乎沒裁到靜止的開頭"
    assert t1 >= 4.0, f"t1={t1:.2f} 切掉了運動的結尾"
    assert t1 <= 5.8, f"t1={t1:.2f} 幾乎沒裁到靜止的結尾"


def test_motion_window_keeps_everything_when_always_moving():
    from pose_log import motion_window

    s = [_s(i / 30.0, i * 0.1) for i in range(60)]
    t0, t1 = motion_window(s)
    assert t0 == pytest.approx(s[0].t)
    assert t1 == pytest.approx(s[-1].t)


def test_motion_window_falls_back_to_full_range_when_never_moving():
    """★ 車完全沒動時要回傳整段，不能回傳空區間 —— 那會錄出 0 幀的影片，
    而且看不出是「沒動」還是「壞掉」。"""
    from pose_log import motion_window

    s = [_s(i / 30.0, 0.0) for i in range(60)]
    t0, t1 = motion_window(s)
    assert (t0, t1) == pytest.approx((s[0].t, s[-1].t))
