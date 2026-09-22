"""程序化步態的測試。

驗的是**步態的物理與對稱性**，不是「看起來像不像」：
真人走路時左右腿反相、同側手腳反相、擺幅隨速度增加。這些都是可量的。
"""

from __future__ import annotations

import math

import pytest

from gait import GAIT_JOINTS, cycle_period, joint_angles, stride_phase


def test_phase_wraps_over_one_cycle():
    """半個週期 → 相位 π；整個週期 → 回到 0。"""
    v = 1.2
    T = cycle_period(v)
    assert stride_phase(0.0, v) == pytest.approx(0.0)
    assert stride_phase(T / 2, v) == pytest.approx(math.pi)
    assert stride_phase(T, v) == pytest.approx(0.0, abs=1e-9)


def test_faster_walking_shortens_the_cycle():
    """步頻隨速度提高 —— 走得快腿擺得快。"""
    assert cycle_period(1.4) < cycle_period(0.6)


def test_cycle_period_is_physically_plausible():
    """成人常速步行的步態週期約 1.0~1.2 s（單腳來回一次）。"""
    assert 0.8 <= cycle_period(1.2) <= 1.4


def test_legs_are_in_antiphase():
    """★ 左右腿必須反相。同相會變成雙腳同時離地的兔子跳。"""
    a = joint_angles(0.0, speed=1.2)
    b = joint_angles(math.pi, speed=1.2)
    assert a["L_thigh"] == pytest.approx(b["R_thigh"], abs=1e-6)
    assert a["R_thigh"] == pytest.approx(b["L_thigh"], abs=1e-6)


def test_arms_swing_opposite_to_the_same_side_leg():
    """★ 同側手腳反相 —— 這是人類步態最顯著的特徵，同相會像機器人。"""
    ang = joint_angles(0.7, speed=1.2)
    assert ang["L_thigh"] * ang["L_upperarm"] < 0


def test_amplitude_grows_with_speed():
    """走得快擺幅大。"""
    slow = max(abs(joint_angles(p, 0.5)["L_thigh"]) for p in (0.0, 1.0, 2.0, 3.0))
    fast = max(abs(joint_angles(p, 1.4)["L_thigh"]) for p in (0.0, 1.0, 2.0, 3.0))
    assert fast > slow


def test_standing_still_has_no_swing():
    ang = joint_angles(1.3, speed=0.0)
    assert all(abs(v) < 1e-9 for v in ang.values())


def test_knee_only_bends_one_way():
    """★ 膝蓋不能反折。任何相位的小腿角度都必須同號。"""
    vals = [joint_angles(p * math.pi / 8, 1.2)["L_calf"] for p in range(16)]
    assert all(v <= 1e-9 for v in vals) or all(v >= -1e-9 for v in vals)


def test_all_gait_joints_are_covered():
    ang = joint_angles(0.4, 1.2)
    assert set(ang) == set(GAIT_JOINTS)


def test_gait_is_periodic():
    a = joint_angles(0.9, 1.2)
    b = joint_angles(0.9 + 2 * math.pi, 1.2)
    for k in a:
        assert a[k] == pytest.approx(b[k], abs=1e-9)


# ---------------------------------------------------------------- 基礎姿勢
from gait import BASE_POSE_RAD, base_pose_angles


def test_arms_are_brought_down_from_the_t_pose():
    """★ NVIDIA People 的綁定姿勢是 T-pose（實測手臂平舉 1.56 m）。
    走路的人手臂垂在身側，所以要先放下來當基礎姿勢。"""
    b = base_pose_angles()
    assert abs(b["L_upperarm"]) > math.radians(60), "左臂沒放下來"
    assert abs(b["R_upperarm"]) > math.radians(60), "右臂沒放下來"


def test_arms_go_down_in_opposite_directions():
    """★ 左右臂從 ±X 各自往下轉，方向必須相反，否則一邊會轉到頭頂。"""
    b = base_pose_angles()
    assert b["L_upperarm"] * b["R_upperarm"] < 0


def test_legs_need_no_base_pose():
    """腿在 T-pose already 指向下方，不需要基礎姿勢修正。"""
    b = base_pose_angles()
    for j in ("L_thigh", "R_thigh", "L_calf", "R_calf"):
        assert b[j] == pytest.approx(0.0)


def test_base_pose_covers_all_gait_joints():
    assert set(base_pose_angles()) == set(GAIT_JOINTS)


def test_base_pose_is_not_a_full_ninety_degrees():
    """完全 90° 會讓手臂緊貼身體、穿模。留一點外張。"""
    assert abs(BASE_POSE_RAD["L_upperarm"]) < math.radians(90)


# ── 相位改由「走過的距離」推進（ORCA 下速度會變）──────────────────────
def test_stride_length_is_speed_times_period():
    from gait import cycle_period, stride_length

    for v in (0.4, 0.7, 1.0):
        assert stride_length(v) == pytest.approx(v * cycle_period(v))


def test_phase_advance_matches_time_based_phase_at_constant_speed():
    """★ 等速時，用距離推進的相位必須與原本用時間算的一致，
    否則換成 ORCA 之後腳步會與位移對不上（滑步）。"""
    import math

    from gait import phase_advance, stride_phase

    for v in (0.5, 0.8, 1.0):
        dt, steps = 1.0 / 60.0, 300
        by_dist = sum(phase_advance(v * dt, v) for _ in range(steps))
        by_time = 2 * math.pi * (steps * dt) / (2 * math.pi / (2 * math.pi)) * 0  # 佔位
        expected = stride_phase(steps * dt, v)
        assert by_dist % (2 * math.pi) == pytest.approx(expected, abs=1e-6)


def test_phase_does_not_advance_when_standing_still():
    """★ 速度 0 時不能推進相位，否則站著的人會原地踏步。"""
    from gait import phase_advance

    assert phase_advance(0.0, 0.0) == 0.0
    assert phase_advance(0.5, 0.0) == 0.0
