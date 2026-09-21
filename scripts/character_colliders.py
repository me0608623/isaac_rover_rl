"""執行期替 People 角色套上**三角網格**碰撞體，讓 PhysX 光達打出人體輪廓。

為什麼必須在執行期做，不能寫進 USD 疊加層：
角色網格藏在 CDN 參照（`https://.../F_Business_02.usd`）底下，離線的 usd-core
沒有 https resolver，`Usd.Stage.Open` 時那些子 prim 根本不存在，疊加層無從
指定路徑。Isaac 的 Omniverse resolver 會把參照解開，所以只有在 stage 載入
之後才找得到 Mesh。

為什麼用 approximation="none"（真三角網格）而不是 convexHull 或 capsule：
使用者要求光達打出來的是**人體形狀**。convexHull 會把手臂與軀幹之間的縫填掉，
capsule 更是直接變成膠囊。三角網格是唯一能保留人體輪廓的近似。

⚠ 已知限制：skinned mesh 的 USD `points` 是綁定姿勢，PhysX cook 一次之後
不會跟著骨架動畫變形。所以碰撞體是「A-pose 的人形」，隨 root motion 平移
旋轉，但四肢不擺動。要連四肢都進點雲需要改用 RTX 光達或每幀重 cook。
"""

from __future__ import annotations

import math

#: 碰撞體與機器人出生點的最小安全距離（m）。
#: kinematic 剛體是無限質量，和車體重疊會把車彈射出去 —— 2026-09-21 實測
#: Character_09 距出生點僅 0.36 m，害 base_footprint 被打到 (-2048,-256,-256)。
MIN_CLEARANCE_FROM_ROBOT_M = 1.5


def too_close_to_robot(char_xy, robot_xy, clearance: float = MIN_CLEARANCE_FROM_ROBOT_M) -> bool:
    """角色是否近到會把機器人彈開。"""
    return math.hypot(char_xy[0] - robot_xy[0], char_xy[1] - robot_xy[1]) < clearance


def apply_mesh_colliders(stage, root_path: str, robot_xy=None,
                         clearance: float = MIN_CLEARANCE_FROM_ROBOT_M):
    """對 ``root_path`` 底下每個角色的 Mesh 套三角網格碰撞體。

    Returns:
        ``(套用的角色數, 套用的 mesh 數, 因太靠近機器人而跳過的角色名單)``
    """
    from pxr import Usd, UsdGeom, UsdPhysics

    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return (0, 0, [])

    cache = UsdGeom.XformCache()
    n_char = n_mesh = 0
    skipped: list[str] = []

    for char in root.GetChildren():
        if char.GetName() == "Biped_Setup":          # 動畫圖設定，不是角色
            continue
        t = cache.GetLocalToWorldTransform(char).ExtractTranslation()
        if robot_xy is not None and too_close_to_robot((t[0], t[1]), robot_xy, clearance):
            skipped.append(char.GetName())
            continue

        applied_here = 0
        for prim in Usd.PrimRange(char, Usd.TraverseInstanceProxies(
                Usd.PrimAllPrimsPredicate)):
            if prim.GetTypeName() != "Mesh":
                continue
            if prim.IsInstanceProxy():               # 實例代理不可編輯
                continue
            UsdPhysics.CollisionAPI.Apply(prim)
            mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
            # "none" = 直接用三角網格，不做凸包近似 —— 保留人體輪廓。
            mc.CreateApproximationAttr().Set("none")
            applied_here += 1

        if applied_here:
            n_char += 1
            n_mesh += applied_here

    return (n_char, n_mesh, skipped)
