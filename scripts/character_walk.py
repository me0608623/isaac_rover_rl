"""把程序化步態寫進角色骨架，讓渲染的人與碰撞體同步擺動。

作法是替每個角色建一個 :class:`UsdSkelAnimation`，綁成 SkelRoot 的
``animationSource``，每幀更新它的 ``rotations``。這樣：

  骨架動 → 渲染的角色動畫 + 我們的部位碰撞體（讀同一個骨架）

兩者自動一致，不需要額外同步。

⚠ ``rotations`` 是關節的**完整局部旋轉**而非增量，所以要先從
``restTransforms`` 取出靜止姿勢再把步態疊上去。

⚠ 旋轉軸不可寫死：骨頭的局部座標系朝向因骨架而異。這裡在建置時把
**世界的左右軸**變換進各關節的局部座標系 —— 髖屈伸、膝彎曲都是繞它。
"""

from __future__ import annotations

import math

from gait import GAIT_JOINTS, base_pose_angles, joint_angles, stride_phase
from skel_parts import joint_to_segment

ANIM_PRIM_NAME = "ProceduralWalk"

#: 世界的「前後軸」與「左右軸」。手臂先繞 FORWARD 放下，再繞 LATERAL 擺動。
FORWARD_AXIS = (0.0, 1.0, 0.0)
LATERAL_AXIS = (1.0, 0.0, 0.0)


def world_delta(base_angle: float, swing_angle: float):
    """回傳「先放下、再擺動」的世界旋轉。

    ⚠ 順序是這整段最容易錯的地方，而且錯了只會讓擺幅變小而不會報錯：
    T-pose 的手臂沿 ±X 伸直，若擺動排在放下**之前**，繞 X 就是繞骨頭自身
    的純扭轉，肘部幾乎不動（2026-09-21 實測行程只剩 4.7 cm，且位移方向
    落在 X-Z 平面而非預期的 Y）。放下之後手臂指向 -Z，繞 X 才與骨頭垂直。

    pxr 是 row-vector：``a * b`` 代表先 a 後 b。
    """
    from pxr import Gf
    base = Gf.Rotation(Gf.Vec3d(*FORWARD_AXIS), math.degrees(base_angle))
    swing = Gf.Rotation(Gf.Vec3d(*LATERAL_AXIS), math.degrees(swing_angle))
    return base * swing


def local_delta(d_world, parent_rot):
    """世界旋轉 → 關節局部旋轉：``D_local = P · D_world · P⁻¹``。

    共軛用**父關節**的世界旋轉，因為 world = local × parent_world。

    ⚠⚠ 全程用 ``GfRotation``，不要混用 ``GfQuatd``：實測 pxr 這兩者的
    乘法順序**相反** —— ``GfRotation`` 的 ``a * b`` 是「先 a 後 b」
    （row-vector），``GfQuatd`` 的 ``qa * qb`` 卻是「先 b 後 a」。
    混用不會拋例外也不會 NaN，只會讓旋轉軸落到錯的平面，表現成
    「有動但擺幅只剩 1/4」（2026-09-21 實測肘部行程 0.047 m vs 應有 0.174 m）。
    見 test_character_walk.test_quat_and_rotation_multiply_in_opposite_orders。
    """
    return parent_rot * d_world * parent_rot.GetInverse()


def _decompose(m):
    """4x4 → (平移, 旋轉四元數, 縮放)。"""
    from pxr import Gf
    t = m.ExtractTranslation()
    r = m.ExtractRotationQuat()
    s = Gf.Vec3h(1.0, 1.0, 1.0)
    return t, r, s


class CharacterWalkDriver:
    """每幀把步態寫進各角色的骨架。"""

    def __init__(self, stage, char_paths, speeds=None):
        from pxr import Gf, Sdf, UsdGeom, UsdSkel, Vt

        self._stage = stage
        self._chars = []
        cache = UsdGeom.XformCache()

        for i, cp in enumerate(char_paths):
            char = stage.GetPrimAtPath(cp)
            if not (char and char.IsValid()):
                continue
            skel_prim = _find_skeleton(stage, char)
            if skel_prim is None:
                continue
            skel = UsdSkel.Skeleton(skel_prim)
            joints = list(skel.GetJointsAttr().Get() or [])
            rest = skel.GetRestTransformsAttr().Get()
            binds = skel.GetBindTransformsAttr().Get()
            if not joints or rest is None or binds is None:
                continue

            # 關節路徑 → 索引，用來查父關節。
            by_path = {jp: i for i, jp in enumerate(joints)}

            # 每個會擺動的關節記下它與**父關節**的世界綁定旋轉。
            # ⚠ 共軛要用父關節而非關節自身：世界旋轉 D 表達成局部旋轉是
            #   D_local = P · D · P⁻¹（P = 父關節的世界旋轉），
            #   因為 world = local × parent_world（pxr 是 row-vector）。
            targets = {}
            for seg in GAIT_JOINTS:
                for ji, jp in enumerate(joints):
                    if joint_to_segment(jp) != seg:
                        continue
                    parent = jp.rsplit("/", 1)[0] if "/" in jp else None
                    pi = by_path.get(parent)
                    prot = (Gf.Matrix4d(binds[pi]).ExtractRotation()
                            if pi is not None else Gf.Rotation(Gf.Vec3d(0, 0, 1), 0))
                    targets[seg] = (ji, prot)
                    break

            anim_path = f"{cp}/{ANIM_PRIM_NAME}"
            anim = UsdSkel.Animation.Define(stage, anim_path)
            anim.CreateJointsAttr(Vt.TokenArray(joints))
            t0, r0, s0 = [], [], []
            for m in rest:
                t, r, s = _decompose(Gf.Matrix4d(m))
                t0.append(Gf.Vec3f(t))
                r0.append(Gf.Quatf(r))
                s0.append(Gf.Vec3h(s))
            anim.CreateTranslationsAttr(Vt.Vec3fArray(t0))
            anim.CreateRotationsAttr(Vt.QuatfArray(r0))
            anim.CreateScalesAttr(Vt.Vec3hArray(s0))

            # 綁成 SkelRoot 的動畫來源
            root = skel_prim.GetParent()
            while root and root.IsValid() and root.GetTypeName() != "SkelRoot":
                root = root.GetParent()
            binding = UsdSkel.BindingAPI.Apply(
                root if (root and root.IsValid()) else skel_prim)
            binding.CreateAnimationSourceRel().SetTargets([Sdf.Path(anim_path)])

            speed = (speeds[i] if speeds and i < len(speeds) else 1.2)
            self._chars.append((anim, list(r0), targets, speed))

    def __len__(self) -> int:
        return len(self._chars)

    def segments_driven(self) -> int:
        return sum(len(c[2]) for c in self._chars)

    def update(self, sim_time: float, phase_offset_per_char: float = 0.7) -> None:
        from pxr import Gf, Vt

        base = base_pose_angles()
        for k, (anim, rest_rot, targets, speed) in enumerate(self._chars):
            phase = stride_phase(sim_time + k * phase_offset_per_char, speed)
            ang = joint_angles(phase, speed)
            rot = list(rest_rot)
            for seg, (ji, prot) in targets.items():
                d_local = local_delta(world_delta(base[seg], ang[seg]), prot)
                # rest 先、delta 後 —— 用 GfRotation 相乘，最後才轉四元數。
                total = Gf.Rotation(Gf.Quatd(rest_rot[ji])) * d_local
                rot[ji] = Gf.Quatf(total.GetQuat())
            anim.GetRotationsAttr().Set(Vt.QuatfArray(rot))


def _find_skeleton(stage, char_prim):
    from pxr import Usd
    for p in Usd.PrimRange(char_prim, Usd.TraverseInstanceProxies(
            Usd.PrimAllPrimsPredicate)):
        if p.GetTypeName() == "Skeleton":
            return p
    return None
