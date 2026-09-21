"""逐部位碰撞體：讓 PhysX 光達打到**會擺動的真實人形**。

背景：PhysX 光達對碰撞體 raycast，而三角網格碰撞體是 cook 一次就固定，
不會跟著骨架變形。整具人體做成一塊，只能停在綁定姿勢 —— NVIDIA People
的綁定姿勢是 T-pose（實測手臂平舉 1.56 m），比圓柱還不像行人。

作法：依骨骼權重把人體網格拆成 10 個部位（見 skel_parts），每塊各自
cook 成三角網格碰撞體，並把頂點存在**該骨骼的局部座標系**裡：

    p_bone = p_skel × B⁻¹        B = 該骨骼的綁定變換（pxr 是 row-vector）

之後每幀只要把骨骼的當前變換寫進該塊的 xform，網格就自動到正確位置 ——
四肢真的會擺動，每塊都保有真實輪廓，而且**不需要重新 cook**，每幀成本
只是剛體變換。

⚠ 部位 prim 掛在 SkelRoot **外面**（角色 Xform 底下），否則 UsdSkel 會試圖
對它做蒙皮，與我們寫入的變換互相打架。
"""

from __future__ import annotations

from skel_parts import (SEGMENTS, assign_faces, assign_vertices,
                        extract_submesh, joint_to_segment)

#: 部位碰撞體掛載處（角色 Xform 底下的子 Scope）。
PARTS_SCOPE = "lidar_parts"

#: 少於這麼多個面的部位不建 —— 太碎的塊 cook 失敗率高，光達也分不出來。
MIN_FACES_PER_PART = 12


def representative_joint(joints: list[str], segment: str) -> int | None:
    """挑一個部位的代表骨骼：骨架順序中**第一個**屬於該部位的關節。

    骨架是由根往外排的，所以第一個命中的就是該部位最靠近軀幹的骨頭
    （例如 L_upperarm 取 L_Upperarm 而不是 L_ElbowShareBone），
    部位繞著它旋轉才符合解剖關係。
    """
    for i, j in enumerate(joints):
        if joint_to_segment(j) == segment:
            return i
    return None


def plan_parts(joints, points_skel, fv_counts, fv_indices,
               joint_indices, joint_weights, elem_size):
    """單一網格版：算出每個部位要用哪些面、綁哪根骨骼。不碰 USD，可離線驗。

    Returns:
        ``{segment: (points, counts, indices, joint_index)}``
    """
    vseg = assign_vertices(joint_indices, joint_weights, elem_size, joints)
    return plan_parts_from_vseg(joints, points_skel, fv_counts, fv_indices, vseg)


def plan_parts_from_vseg(joints, points_skel, fv_counts, fv_indices, vseg):
    """多網格版：頂點歸屬已先算好再傳進來。

    ⚠ 角色由 5 個網格組成，每個網格的 elementSize 不同（實測 3/5/6/7/5），
    權重陣列無法直接串接 —— 必須各自算完歸屬再合併。
    """
    fseg = assign_faces(fv_counts, fv_indices, vseg)

    out = {}
    for seg in SEGMENTS:
        ji = representative_joint(joints, seg)
        if ji is None:
            continue
        p, c, i = extract_submesh(points_skel, fv_counts, fv_indices, fseg, seg)
        if len(c) < MIN_FACES_PER_PART:
            continue
        out[seg] = (p, c, i, ji)
    return out


class CharacterPartsDriver:
    """每幀把各部位碰撞體挪到對應骨骼當前的位置。"""

    def __init__(self, stage, char_paths: list[str]):
        from pxr import UsdGeom, UsdSkel

        self._stage = stage
        self._entries = []          # (part_xform_op, skel_query, joint_index, skel_prim, char_prim)
        self._cache = UsdGeom.XformCache()
        self._skel_cache = UsdSkel.Cache()

        for cp in char_paths:
            char = stage.GetPrimAtPath(cp)
            scope = stage.GetPrimAtPath(f"{cp}/{PARTS_SCOPE}")
            if not (char and char.IsValid() and scope and scope.IsValid()):
                continue
            skel_prim = _find_skeleton(stage, char)
            if skel_prim is None:
                continue
            sq = self._skel_cache.GetSkelQuery(UsdSkel.Skeleton(skel_prim))
            for part in scope.GetChildren():
                ji = part.GetAttribute("charge:jointIndex")
                if not ji or ji.Get() is None:
                    continue
                xf = UsdGeom.Xformable(part)
                ops = [o for o in xf.GetOrderedXformOps()
                       if o.GetOpType() == UsdGeom.XformOp.TypeTransform]
                if ops:
                    self._entries.append((ops[0], sq, int(ji.Get()),
                                          skel_prim, char))

    def __len__(self) -> int:
        return len(self._entries)

    def update(self) -> None:
        from pxr import Usd

        self._cache.Clear()
        cached: dict[int, list] = {}
        for op, sq, ji, skel_prim, char in self._entries:
            key = id(sq)
            xforms = cached.get(key)
            if xforms is None:
                xforms = sq.ComputeJointSkelTransforms(Usd.TimeCode.Default())
                cached[key] = xforms
            if not xforms or ji >= len(xforms):
                continue
            # 部位掛在角色 Xform 底下，所以要寫的是「相對角色」的變換：
            #   p_char = p_bone × jointSkelXf × M(skel→char)
            skel_to_char = self._cache.ComputeRelativeTransform(skel_prim, char)[0]
            op.Set(xforms[ji] * skel_to_char)


def _find_skeleton(stage, char_prim):
    from pxr import Usd
    for p in Usd.PrimRange(char_prim, Usd.TraverseInstanceProxies(
            Usd.PrimAllPrimsPredicate)):
        if p.GetTypeName() == "Skeleton":
            return p
    return None


def build_character_parts(stage, char_path: str, invisible: bool = True,
                          approximation: str = "convexHull"):
    """替一個角色建立逐部位三角網格碰撞體。

    Args:
        approximation: PhysX 的碰撞近似。
            ``"none"``       真三角網格，輪廓最精確但最貴（實測 19 人 34 萬面
                             → RTF 0.44x，只有半速）。
            ``"convexHull"`` 每塊取凸包（預設）。**逐部位之後凸包才可用** ——
                             前臂、大腿、軀幹各自近似凸形，取凸包幾乎不損輪廓；
                             整具人體取凸包才會把腋下填平變成一團。

    Returns:
        ``(建立的部位數, 總面數, 訊息)``
    """
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdSkel, Vt

    char = stage.GetPrimAtPath(char_path)
    if not (char and char.IsValid()):
        return (0, 0, "角色 prim 不存在")
    skel_prim = _find_skeleton(stage, char)
    if skel_prim is None:
        return (0, 0, "找不到 Skeleton")

    skel = UsdSkel.Skeleton(skel_prim)
    joints = list(skel.GetJointsAttr().Get() or [])
    binds = skel.GetBindTransformsAttr().Get()
    if not joints or not binds:
        return (0, 0, "骨架缺 joints 或 bindTransforms")

    all_pts, all_counts, all_idx, all_vseg = [], [], [], []
    offset = 0
    for prim in Usd.PrimRange(char, Usd.TraverseInstanceProxies(
            Usd.PrimAllPrimsPredicate)):
        if prim.GetTypeName() != "Mesh" or prim.IsInstanceProxy():
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idx:
            continue
        b = UsdSkel.BindingAPI(prim)
        ji, jw = b.GetJointIndicesAttr().Get(), b.GetJointWeightsAttr().Get()
        if ji is None or jw is None:
            continue
        elem = b.GetJointIndicesPrimvar().GetElementSize() or 1
        gbt = b.GetGeomBindTransformAttr().Get() or Gf.Matrix4d(1.0)

        # 網格局部 → skel space（pxr 是 row-vector：p' = p × M）
        all_pts.extend(gbt.Transform(Gf.Vec3f(p)) for p in pts)
        all_vseg.extend(assign_vertices(list(ji), list(jw), elem, joints))
        all_counts.extend(int(c) for c in counts)
        all_idx.extend(int(i) + offset for i in idx)
        offset += len(pts)

    if not all_pts:
        return (0, 0, "沒有可用的蒙皮網格")

    plan = plan_parts_from_vseg(joints, all_pts, all_counts, all_idx, all_vseg)
    scope_path = f"{char_path}/{PARTS_SCOPE}"
    if not stage.GetPrimAtPath(scope_path).IsValid():
        UsdGeom.Scope.Define(stage, scope_path)

    n_part = n_face = 0
    for seg, (pts, counts, idx, ji) in plan.items():
        # 頂點改存在骨骼的局部座標系：p_bone = p_skel × B⁻¹
        # 這樣每幀只要寫入該骨骼當前變換，網格自動到位，不必重新 cook。
        binv = Gf.Matrix4d(binds[ji]).GetInverse()
        local = [binv.Transform(Gf.Vec3f(p)) for p in pts]

        path = f"{scope_path}/{seg}"
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(p) for p in local]))
        m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        xs = [p[0] for p in local]; ys = [p[1] for p in local]; zs = [p[2] for p in local]
        m.CreateExtentAttr([(min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))])
        m.MakeMatrixXform().Set(Gf.Matrix4d(1.0))
        if invisible:
            # 角色網格本身已經在渲染了，碰撞體不需要再畫一層。
            UsdGeom.Imageable(m.GetPrim()).CreateVisibilityAttr(UsdGeom.Tokens.invisible)

        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        mc = UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim())
        mc.CreateApproximationAttr().Set(approximation)
        body = UsdPhysics.RigidBodyAPI.Apply(m.GetPrim())
        body.CreateKinematicEnabledAttr(True)
        m.GetPrim().CreateAttribute("charge:jointIndex", Sdf.ValueTypeNames.Int,
                                    custom=True).Set(int(ji))
        n_part += 1
        n_face += len(counts)

    return (n_part, n_face, f"{n_part} 部位 / {n_face} 面")
