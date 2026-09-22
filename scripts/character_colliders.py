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

#: 機器人的 articulation root。碰撞過濾掛在這裡可一次涵蓋所有 link。
ROBOT_ARTICULATION_PATH = "/World/charger_rover4_5_0/charger_rover_urdf5"

#: 碰撞體與機器人出生點的最小安全距離（m）。
#: kinematic 剛體是無限質量，和車體重疊會把車彈射出去 —— 2026-09-21 實測
#: Character_09 距出生點僅 0.36 m，害 base_footprint 被打到 (-2048,-256,-256)。
MIN_CLEARANCE_FROM_ROBOT_M = 1.5


def too_close_to_robot(char_xy, robot_xy, clearance: float = MIN_CLEARANCE_FROM_ROBOT_M) -> bool:
    """角色是否近到會把機器人彈開。"""
    return math.hypot(char_xy[0] - robot_xy[0], char_xy[1] - robot_xy[1]) < clearance


#: 站立角色與 routing 站點的最小淨空（m，map frame）。
#: 站著的人立在導航點位上會擋住目標：2026-09-22 實測 Character_15 離 c24
#: 只有 0.40 m、Character_09 離 c28 只有 0.36 m（後者剛好是錄影的起點）。
#: 1.0 m 取自 monitor_navigation.ARRIVE_RADIUS_M —— 抵達判定半徑內不該站人。
#: ⚠ 只管**站著**的角色。會走的角色路線本來就沿走廊、必然經過站點，
#: 那是刻意設計的動態互動，不適用這條規則。
MIN_CLEARANCE_FROM_ROUTING_NODE_M = 1.0


def nearest_routing_node(map_xy, stations):
    """回傳離 ``map_xy`` 最近的 routing 站點 ``(站名, 距離)``（map frame）。

    站點表是空的時回傳 ``(None, inf)`` —— 讀不到表要當成「沒有限制」，
    不能讓呼叫端把全部角色都當成違規停用掉。
    """
    best, bd = None, float("inf")
    for name, pose in stations.items():
        d = math.hypot(pose[0] - map_xy[0], pose[1] - map_xy[1])
        if d < bd:
            best, bd = name, d
    return best, bd


def too_close_to_routing_node(map_xy, stations,
                              clearance: float = MIN_CLEARANCE_FROM_ROUTING_NODE_M) -> bool:
    """這個位置是否壓在某個 routing 站點上。"""
    return nearest_routing_node(map_xy, stations)[1] < clearance


#: 站立角色彼此的最小間距（m）。2026-09-22 使用者指定「減少行人在一起的密度」。
MIN_SPACING_BETWEEN_STANDING_M = 2.5


def thin_by_spacing(items, min_spacing: float = MIN_SPACING_BETWEEN_STANDING_M):
    """把擠在一起的角色挑掉，回傳 ``(保留的名字, 丟掉的名字)``。

    貪心法：依**名字排序**後逐一檢查，離已保留者都夠遠才留。

    ⚠ 排序是刻意的，不是為了好看：第一遍（導航）與第二遍（回放算圖）
    是兩個不同的行程，必須留下**同一批人**，否則影片裡的人數會與 rosbag
    對不起來。輸入順序若來自 USD 走訪，兩遍不保證一致。
    """
    kept: list[str] = []
    kept_xy: list[tuple[float, float]] = []
    dropped: list[str] = []
    for name, xy in sorted(items, key=lambda it: it[0]):
        if any(math.hypot(xy[0] - k[0], xy[1] - k[1]) < min_spacing for k in kept_xy):
            dropped.append(name)
            continue
        kept.append(name)
        kept_xy.append(xy)
    return kept, dropped


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


def filter_contacts_with_robot(stage, prim, robot_path: str = ROBOT_ARTICULATION_PATH) -> bool:
    """關閉 ``prim`` 與機器人之間的**接觸力**，但保留光達 raycast。

    ⚠ 為什麼非做不可：行人是 kinematic 剛體 = **無限質量**。任何與機器人
    articulation 的接觸都會把車彈射出去 —— 2026-09-21 實測導航途中車體
    物理爆掉，/odom_gt 與點雲全變成 NaN。出生點的安全距離檢查只擋得住
    起始重疊，擋不住「走動中的行人撞上移動中的機器人」，而迎面互動
    正是刻意設計的場景，碰撞必然發生。

    ⚠ 為什麼不是「拿掉碰撞體」：PhysX 光達是對碰撞體 raycast，拿掉就看不到了。
    FilteredPairsAPI 只過濾**接觸產生**，場景查詢（raycast）走另一條路徑，
    所以行人仍會出現在點雲裡。

    導航評測本來就不需要模擬撞擊反應 —— 碰撞用光達最近距離判定即可
    （見 monitor_navigation.COLLISION_RANGE_M）。
    """
    from pxr import Sdf, UsdPhysics
    if not (prim and prim.IsValid()):
        return False
    fp = UsdPhysics.FilteredPairsAPI.Apply(prim)
    fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(robot_path))
    return True
