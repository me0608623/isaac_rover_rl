"""旋轉組成的離線測試（只需 usd-core，不必啟動 Isaac）。

為什麼值得單獨測：順序錯了**不會報錯**，只會讓擺幅變小 —— 實測第一版
肘部行程只剩 4.7 cm（應為約 19 cm），而且要開 Isaac 兩分鐘才量得到。
這裡用一根假手臂把幾何直接算出來，毫秒級就能驗。
"""

from __future__ import annotations

import math

import pytest
from pxr import Gf

from character_walk import local_delta, world_delta

#: T-pose 的左上臂：肩在原點，肘沿 +X 伸出 0.27 m。
ELBOW_T_POSE = Gf.Vec3d(0.27, 0.0, 0.0)
ARM_DOWN = math.radians(78.0)
SWING = math.radians(20.0)
IDENTITY = Gf.Rotation(Gf.Vec3d(0, 0, 1), 0.0)


def _elbow_after(base, swing):
    d = world_delta(base, swing)
    r = local_delta(d, IDENTITY)      # 回傳 GfRotation（見 local_delta 的說明）
    return r.TransformDir(ELBOW_T_POSE)


def test_base_pose_brings_the_elbow_down():
    """78° 放下後，肘部應該主要在下方（-Z），而不是還平舉在 +X。"""
    e = _elbow_after(ARM_DOWN, 0.0)
    assert e[2] < -0.2, f"肘部沒降下來: {e}"
    assert abs(e[0]) < 0.12, f"肘部仍平舉: {e}"


def test_swing_moves_the_elbow_fore_and_aft():
    """★ 放下後繞左右軸擺動，肘部必須主要沿**前後方向(Y)**移動。

    第一版的 bug 正是位移落在 X-Z 平面（繞 Y），代表擺動被排在放下之前，
    變成繞骨頭自身的扭轉。
    """
    fwd = _elbow_after(ARM_DOWN, +SWING)
    aft = _elbow_after(ARM_DOWN, -SWING)
    dy = abs(fwd[1] - aft[1])
    dx = abs(fwd[0] - aft[0])
    assert dy > dx, f"擺動方向錯誤：Δy={dy:.3f} 應大於 Δx={dx:.3f}"


def test_swing_amplitude_matches_the_geometry():
    """★ ±20° 配 0.27 m 上臂 → 峰對峰 2·L·sin(20°) ≈ 0.185 m。

    實測第一版只有 0.047 m（1/4），就是順序錯導致的扭轉殘量。
    """
    fwd = _elbow_after(ARM_DOWN, +SWING)
    aft = _elbow_after(ARM_DOWN, -SWING)
    span = (Gf.Vec3d(fwd) - Gf.Vec3d(aft)).GetLength()
    expected = 2 * ELBOW_T_POSE.GetLength() * math.sin(SWING)
    assert span == pytest.approx(expected, rel=0.15), \
        f"擺幅 {span:.3f} m 與幾何預期 {expected:.3f} m 不符"


def test_zero_swing_is_pure_base_pose():
    a = _elbow_after(ARM_DOWN, 0.0)
    b = _elbow_after(ARM_DOWN, 0.0)
    assert Gf.Vec3d(a) == pytest.approx(tuple(b), abs=1e-12)


def test_no_rotation_leaves_the_t_pose_untouched():
    e = _elbow_after(0.0, 0.0)
    assert Gf.Vec3d(e) == pytest.approx(tuple(ELBOW_T_POSE), abs=1e-9)


def test_conjugation_by_parent_is_identity_when_parent_is_identity():
    d = world_delta(ARM_DOWN, SWING)
    r = local_delta(d, IDENTITY)
    v = Gf.Vec3d(0.3, -0.2, 0.5)
    assert Gf.Vec3d(r.TransformDir(v)) == pytest.approx(
        tuple(d.TransformDir(v)), abs=1e-9)


# ---------------------------------------------- pxr 乘法順序陷阱
def test_quat_and_rotation_multiply_in_opposite_orders():
    """★ 釘住 pxr 的慣例差異，避免未來有人「順手」改回四元數相乘。

    GfRotation 的 a*b 是「先 a 後 b」，GfQuatd 的 qa*qb 卻是「先 b 後 a」。
    混用不報錯、不 NaN，只讓擺幅默默變成 1/4。
    """
    X, Y = Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0)
    r1, r2 = Gf.Rotation(Y, 90.0), Gf.Rotation(X, 90.0)
    v = Gf.Vec3d(1, 0, 0)
    first_then_second = r2.TransformDir(r1.TransformDir(v))

    by_rotation = (r1 * r2).TransformDir(v)
    by_quat = Gf.Rotation(Gf.Quatd(r1.GetQuat()) *
                          Gf.Quatd(r2.GetQuat())).TransformDir(v)

    assert Gf.Vec3d(by_rotation) == pytest.approx(
        tuple(first_then_second), abs=1e-9), "GfRotation 應為「先 a 後 b」"
    assert Gf.Vec3d(by_quat) != pytest.approx(
        tuple(first_then_second), abs=1e-6), "GfQuatd 應為相反順序"


def test_conjugation_with_a_real_non_identity_parent():
    """★ 用真實骨架的父關節旋轉（L_Clavicle，110.9°）驗共軛。

    先前的測試只用單位父旋轉，P·D·P⁻¹ 退化成 D，完全繞過了共軛與
    四元數乘法 —— 測試全綠卻沒保護到，實機才發現擺幅不足。
    """
    parent = Gf.Rotation(Gf.Vec3d(0.56, 0.66, -0.51).GetNormalized(), 110.9)
    d = world_delta(ARM_DOWN, SWING)
    dl = local_delta(d, parent)

    # 骨架把局部旋轉累積成世界：world = local × parent（row-vector）。
    # 取 rest = 單位，則關節的世界靜止朝向就是 P，所以對一個**局部**向量
    # v 而言，「先用 P 送到世界、再於世界施加 D」必須等於「直接用
    # (d_local × P)」—— 這正是共軛要保證的性質。
    v_local = Gf.Vec3d(0.27, 0.0, 0.0)
    via_conjugate = (dl * parent).TransformDir(v_local)
    expected = d.TransformDir(parent.TransformDir(v_local))
    assert Gf.Vec3d(via_conjugate) == pytest.approx(tuple(expected), abs=1e-9), \
        "共軛後施加於世界的旋轉必須與原本的世界旋轉一致"
