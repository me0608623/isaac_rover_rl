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

from character_path import (character_pose_at, facing_rotation_deg,
                            foot_offset_from_bbox, origin_z_for_feet_on_floor)
from gait import GAIT_JOINTS, base_pose_angles, joint_angles, stride_phase
from ros_graph_spec import map_to_world as S_map_to_world
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

    def __init__(self, stage, char_paths, walks=None, floor_top: float = 0.0):
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel, Vt

        self._stage = stage
        self._chars = []
        #: 幾乎靜止時沿用上一次朝向，避免行人原地亂轉。
        self._last_yaw: dict = {}
        self._floor_top = floor_top
        by_name = {w.name: w for w in (walks or ())}
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

            # ⚠ 只對「有路徑」的角色接管 transform：MakeMatrixXform() 會清掉
            #   既有的 xform op stack，對站著不動的角色會把它重設成單位變換，
            #   所有人就會擠到世界原點。
            # ⚠ 角色原點**不在腳底**（實測在腳底上方 0.119~0.151 m，每人不同）。
            #   不量就照 floor_top 擺，腳會陷進地板 12~15 cm。
            _bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                    [UsdGeom.Tokens.default_,
                                     UsdGeom.Tokens.render])
            _r = _bb.ComputeWorldBound(char).ComputeAlignedRange()
            _oz = cache.GetLocalToWorldTransform(char).ExtractTranslation()[2]
            foot = (foot_offset_from_bbox(_oz, _r.GetMin()[2])
                    if not _r.IsEmpty() else 0.0)

            walk = by_name.get(char.GetName())
            op = None
            if walk is not None and walk.waypoints:
                op = UsdGeom.Xformable(char).MakeMatrixXform()
            speed = walk.speed if walk is not None else 0.0
            self._chars.append((anim, list(r0), targets, speed, walk, op,
                                char.GetName(), foot))

    def place_standing(self, name: str, map_xy, yaw: float) -> bool:
        """把站著的角色擺到指定的 map 位置與朝向（腳底貼地）。

        ⚠ 與 ground_standing 一樣不可以用 MakeMatrixXform() —— 會清掉既有
        xform op stack，站著的角色會被重設成單位變換、擠到世界原點。
        """
        from pxr import Gf, UsdGeom

        for c in self._chars:
            op, nm, foot = c[5], c[6], c[7]
            if nm != name:
                continue
            if op is not None:
                return False        # 會走的角色不該被硬擺
            prim = self._stage.GetPrimAtPath(f"/World/Characters/{name}")
            if not (prim and prim.IsValid()):
                return False
            wx, wy, wyaw = S_map_to_world(map_xy[0], map_xy[1], yaw)
            wz = origin_z_for_feet_on_floor(self._floor_top, foot)
            m = Gf.Matrix4d(1.0).SetRotate(
                Gf.Rotation(Gf.Vec3d(0, 0, 1), facing_rotation_deg(wyaw)))
            m = m * Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(wx, wy, wz))
            ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
            for o in ops:
                if o.GetOpType() == UsdGeom.XformOp.TypeTransform:
                    o.Set(m)
                    self._last_yaw[name] = yaw
                    return True
            # 沒有 transform op：退而只改平移與旋轉分開的那兩個 op
            done_t = done_r = False
            for o in ops:
                if o.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    o.Set(Gf.Vec3d(wx, wy, wz)); done_t = True
                elif o.GetOpType() in (UsdGeom.XformOp.TypeOrient,
                                       UsdGeom.XformOp.TypeRotateZ,
                                       UsdGeom.XformOp.TypeRotateXYZ):
                    if o.GetOpType() == UsdGeom.XformOp.TypeRotateZ:
                        o.Set(facing_rotation_deg(wyaw))
                        done_r = True
                    elif o.GetOpType() == UsdGeom.XformOp.TypeOrient:
                        q = Gf.Rotation(Gf.Vec3d(0, 0, 1),
                                        facing_rotation_deg(wyaw)).GetQuat()
                        # ⚠ orient op 的精度可能是 quatf 或 quatd，型別不符
                        #   Set() 會丟 Tf.ErrorException。照 op 自己的型別給。
                        for ctor in (Gf.Quatf, Gf.Quatd, Gf.Quath):
                            try:
                                o.Set(ctor(q))
                                done_r = True
                                break
                            except Exception:      # noqa: BLE001
                                continue
            self._last_yaw[name] = yaw
            return done_t or done_r
        return False

    def ground_standing(self) -> int:
        """把**站著不動**的角色降到地板上（會走的由 update 每幀處理）。

        ⚠ 不可以用 MakeMatrixXform()：那會清掉既有的 xform op stack，
        站著的角色會被重設成單位變換、全部擠到世界原點
        （2026-09-21 踩過，見 __init__ 裡同一個警告）。
        這裡只改既有 translate/transform op 的 z。
        """
        from pxr import Gf, UsdGeom

        n = 0
        for anim, _r, _t, _s, _w, op, name, foot in self._chars:
            if op is not None:          # 會走的，由 update 逐幀擺
                continue
            prim = self._stage.GetPrimAtPath(
                f"/World/Characters/{name}")
            if not (prim and prim.IsValid()):
                continue
            z = origin_z_for_feet_on_floor(self._floor_top, foot)
            ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
            done = False
            for o in ops:
                if o.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    v = o.Get()
                    o.Set(Gf.Vec3d(v[0], v[1], z))
                    done = True
                    break
                if o.GetOpType() == UsdGeom.XformOp.TypeTransform:
                    m = Gf.Matrix4d(o.Get())
                    t = m.ExtractTranslation()
                    m.SetTranslateOnly(Gf.Vec3d(t[0], t[1], z))
                    o.Set(m)
                    done = True
                    break
            if done:
                n += 1
        return n

    def __len__(self) -> int:
        return len(self._chars)

    def last_yaw(self, name: str) -> float:
        """上一次用過的朝向（rad，map frame）。行人幾乎靜止時沿用它。"""
        return self._last_yaw.get(name, 0.0)

    def segments_driven(self) -> int:
        return sum(len(c[2]) for c in self._chars)

    def walking(self) -> int:
        """實際沿路徑移動的角色數（其餘站著，仍套基礎姿勢）。"""
        return sum(1 for c in self._chars if c[5] is not None)

    def update(self, sim_time: float, phase_offset_per_char: float = 0.7,
               poses=None) -> None:
        """把姿勢與位置寫進骨架。

        Args:
            poses: ``{角色名: (map_x, map_y, yaw_rad_or_None, phase, speed)}``。
                給了就照它擺（ORCA 驅動或第二遍回放）；``None`` 時退回原本
                「位置與相位都是模擬時間的函式」的等速往返行為。
        """
        from pxr import Gf, Vt

        base = base_pose_angles()
        for k, (anim, rest_rot, targets, speed, walk, op, name, foot) in enumerate(self._chars):
            ext = poses.get(name) if poses else None

            # 沿路徑移動：角色原點在腳底，z 直接取地板高度。
            if op is not None:
                if ext is not None:
                    mx, my, myaw, phase, speed = ext
                    if myaw is None:                 # 幾乎靜止 → 沿用上一次朝向
                        myaw = self._last_yaw.get(name, 0.0)
                    self._last_yaw[name] = myaw
                    wx, wy, wyaw = S_map_to_world(mx, my, myaw)
                    wz = origin_z_for_feet_on_floor(self._floor_top, foot)
                else:
                    wx, wy, wz, wyaw = character_pose_at(
                        walk, sim_time, self._floor_top, foot)
                m = Gf.Matrix4d(1.0).SetRotate(
                    Gf.Rotation(Gf.Vec3d(0, 0, 1), facing_rotation_deg(wyaw)))
                m = m * Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(wx, wy, wz))
                op.Set(m)

            if ext is not None:
                phase, speed = ext[3], ext[4]
            else:
                offset = walk.phase_s if walk is not None else k * phase_offset_per_char
                phase = stride_phase(sim_time + offset, speed)
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
