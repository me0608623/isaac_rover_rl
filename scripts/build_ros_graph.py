#!/usr/bin/env python3
"""把 ros_graph_spec 的規格套用到 3floor USD，產出一份 USD override layer。

設計要點
--------
1. **原始 USD 一個位元都不動**。所有修正寫進一份新的 ASCII ``.usda``，
   該檔以 subLayer 方式疊在原始 USD 上。Isaac Sim 開這份 override 檔即可。
2. **純 usd-core 實作，不需要 Isaac Sim runtime**。OmniGraph 節點在 USD 裡
   只是帶 ``node:type`` 的普通 prim，可以直接 author。
3. 輸出是 ASCII，**可以 git diff、可以 code review**。

用法
----
    python3 sim_ws/scripts/build_ros_graph.py \
        --input  /home/aa/Ros/charge_rl/assets/3F/3floor_ver_1.usd \
        --output sim_ws/assets/3floor_ver_1_ros_fixed.usda

    # 只檢查目前 override 是否仍與 spec 一致（CI 用）
    python3 sim_ws/scripts/build_ros_graph.py --verify sim_ws/assets/3floor_ver_1_ros_fixed.usda

對照文件：docs/2026-09-21_模擬ROS契約對照_PC端回覆.md
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom

import ros_graph_spec as S

# --------------------------------------------------------------------------
# USD 路徑常數 —— 取自原始檔實測，不是猜的
# --------------------------------------------------------------------------

ROBOT = "/World/charger_rover4_5_0/charger_rover_urdf5"
G_ODOM = f"{ROBOT}/transform_tree_odometry"
G_VELO = f"{ROBOT}/velodyne_01"
G_IMU = f"{ROBOT}/chassis_imu"
G_2D = f"{ROBOT}/ros_lidars"
G_DRIVE = f"{ROBOT}/differential_drive"

P_BASE_LINK = f"{ROBOT}/base_link"
P_BASE_FOOTPRINT = f"{ROBOT}/base_footprint"
P_ROBOT_ROOT = "/World/charger_rover4_5_0"
P_VELO_JOINT = f"{ROBOT}/velodyne_base_mount_joint"
P_FOOTPRINT_JOINT = f"{ROBOT}/base_base_footprint"
P_VELODYNE = f"{ROBOT}/velodyne"
P_LIDAR = f"{ROBOT}/velodyne/Lidar"

#: TF 發佈節點。方案 A（Isaac 不發任何 TF）下全部停用。
TF_PUBLISHER_NODES = (
    f"{G_ODOM}/ros2_publish_raw_transform_tree",
    f"{G_ODOM}/tf_tree_base_link_to_sensors",
    f"{G_ODOM}/tf_tree_base_link_to_wheel_base",
    f"{G_ODOM}/tf_tree_base_link_to_base_footprint",
)

#: 原始檔把所有 timeStamp 接到 OnPlaybackTick.outputs:time（播放時間），
#: 而每張 graph 裡的 IsaacReadSimulationTime 是懸空的。必須改接模擬時間，
#: 否則點雲時戳與 /clock 不同源，NDT 的 lookupTransform 會全數丟 extrapolation。
STAMP_REWIRES = (
    (f"{G_ODOM}/ros2_publish_odometry", f"{G_ODOM}/isaac_read_simulation_time"),
    (f"{G_VELO}/ros2_publish_point_cloud", f"{G_VELO}/isaac_read_simulation_time"),
    (f"{G_IMU}/ros2_publish_imu", f"{G_IMU}/isaac_read_simulation_time"),
)

CLOCK_GRAPH = "/World/ROS_Clock"

#: 模擬加入的走廊障礙物放這裡（與場景原有幾何分開，好辨識與刪除）。
OBSTACLE_ROOT = "/World/SimObstacles"
CHARACTER_ROOT = "/World/Characters"
MAP_PATCH_PATH = "/World/MapPatch"

#: TF 停用的雙保險：導到沒人訂閱的 topic。
DEAD_TF_TOPIC = "/isaac_tf_disabled"

#: 前後 2D 光達（RPLIDAR S2E）的節點。
#: 2026-09-21 實測 /scan 與 /back_scan **完全沒有資料** —— RPLidar_S2E 的 USD 資產
#: 是抓不到的 S3 連結，RTX render product 建不起來。但這些節點每幀仍在嘗試建
#: render product，白白吃 GPU。RL policy 只吃 /velodyne_points，不需要它們。
TWO_D_LIDAR_NODES = (
    f"{G_2D}/publish_front_2d_lidar_scan",
    f"{G_2D}/publish_back_2d_lidar_scan",
    f"{G_2D}/front_2d_lidar_render_product",
    f"{G_2D}/back_2d_lidar_render_product",
    f"{G_2D}/isaac_run_one_simulation_frame",
)

#: 行人 navmesh 的輔助地板（152x24 / 24x32 / 24x28 m 的大板）。
#: 它們同時是**走廊大部分區域唯一的可行走碰撞面** —— 走廊 mesh Mesh_015 只在
#: 部分區域有地板幾何（實測 World(-15.4,+7.8) 有、(-17.3,+7.8) 沒有）。
#: 但預設高度（上表面 -0.46）比真實地板（-0.323）低 13.7 cm，
#: PhysX 光達打得到 → 點雲出現第二層假地板。
#: 正解是**抬高對齊**而非關閉碰撞：
#:   關閉碰撞 → 車開到 Mesh_015 沒覆蓋的地方就掉出世界（實測掉到 z=-7102 m）
NAV_FLOOR_PRIMS = ("/World/NavFloor_H", "/World/NavFloor_V1", "/World/NavFloor_V2")

#: 走廊 mesh 的地板上表面，用來對齊 NavFloor。
CORRIDOR_MESH = "/World/Env_0/Floor/_F/Mesh_015"


@dataclass
class Change:
    """一筆修改紀錄，用於輸出報告。"""

    target: str
    field: str
    before: object
    after: object

    def __str__(self) -> str:
        return f"  {self.target}\n      {self.field}: {self.before!r} -> {self.after!r}"


# --------------------------------------------------------------------------
# 基礎操作
# --------------------------------------------------------------------------

def _require(stage: Usd.Stage, path: str) -> Usd.Prim:
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        raise LookupError(f"USD 缺少預期的 prim: {path}（原始檔版本可能不符）")
    return prim


def _set_attr(stage: Usd.Stage, prim_path: str, name: str, value, type_name) -> Change | None:
    """在 override layer 上設定屬性值。值相同則不寫（維持 idempotent）。"""
    prim = _require(stage, prim_path)
    attr = prim.GetAttribute(name)
    before = attr.Get() if attr and attr.IsValid() else None
    if before == value:
        return None
    if not attr or not attr.IsValid():
        attr = prim.CreateAttribute(name, type_name, custom=True)
    attr.Set(value)
    return Change(prim_path, name, before, value)


# --------------------------------------------------------------------------
# 各項修正
# --------------------------------------------------------------------------

def _disable_nodes(stage: Usd.Stage, node_paths, reason: str) -> list[Change]:
    """停用 OmniGraph 節點。

    ⚠ **不能用 prim.SetActive(False)** —— 2026-09-21 實測：usd-core 讀到的是
    inactive、單元測試也過，但 Isaac Sim 照樣把節點實體化並發佈 topic。
    OmniGraph 不看 prim 的 active 狀態。

    可靠的做法是斷開 ``inputs:execIn``：action graph 的節點沒有 exec 輸入
    就永遠不會被求值。對有 ``inputs:enabled`` 的節點再多關一道。
    """
    changes: list[Change] = []
    for path in node_paths:
        prim = _require(stage, path)
        attr = prim.GetAttribute("inputs:execIn")
        if attr and attr.IsValid():
            before = [str(c) for c in attr.GetConnections()]
            if before:
                attr.SetConnections([])           # 明確空列表，蓋掉弱層的意見
                changes.append(Change(path, f"inputs:execIn 斷開（{reason}）", before, []))
        en = prim.GetAttribute("inputs:enabled")
        if en and en.IsValid() and en.Get() is not False:
            en.Set(False)
            changes.append(Change(path, "inputs:enabled", en.Get(), False))
    return changes


def disable_isaac_tf_publishers(stage: Usd.Stage, spec: S.SimRosSpec) -> list[Change]:
    """方案 A：Isaac 一條 TF 都不發，全部交給 injector 與 robot_state_publisher。

    USD 的 subLayer 無法真正刪除 prim，改用 SetActive(False)。
    停用的 prim 不會被 OmniGraph 實體化。
    """
    own = spec.tf_ownership
    if own.isaac_publishes_odom_tf or own.isaac_publishes_sensor_tf or own.isaac_publishes_footprint_tf:
        raise NotImplementedError(
            "目前僅實作方案 A（Isaac 不發 TF）。若要改成 Isaac 發 TF，"
            "必須同時處理 frame 命名與 staticPublisher 旗標。"
        )
    changes = _disable_nodes(stage, TF_PUBLISHER_NODES, "方案 A：Isaac 不發 TF")
    # 雙保險：即使 execIn 斷開失效，TF 也只會流向沒人訂閱的死路，不污染 tf2 樹
    for node in TF_PUBLISHER_NODES:
        c = _set_attr(stage, node, "inputs:topicName", DEAD_TF_TOPIC, Sdf.ValueTypeNames.String)
        if c:
            changes.append(c)
    return changes


def retarget_odometry_topic(stage: Usd.Stage, spec: S.SimRosSpec) -> list[Change]:
    """模擬發 /odom_gt（真值），由 odom_drift_injector 劣化成 /odom。

    若模擬直接佔用 /odom，injector 無處插入，漂移注入整個失效（交接單 §5.4）。
    """
    node = f"{G_ODOM}/ros2_publish_odometry"
    return [c for c in (
        _set_attr(stage, node, "inputs:topicName", spec.topics.odom_ground_truth, Sdf.ValueTypeNames.String),
        # injector 會覆寫 frame_id / child_frame_id，這裡設對只是為了讓
        # 未接 injector 時直接看 /odom_gt 也語意正確。
        _set_attr(stage, node, "inputs:odomFrameId", spec.frames.odom, Sdf.ValueTypeNames.String),
        _set_attr(stage, node, "inputs:chassisFrameId", spec.frames.base_footprint, Sdf.ValueTypeNames.String),
    ) if c]


def fix_sensor_frame_ids(stage: Usd.Stage, spec: S.SimRosSpec) -> list[Change]:
    """原始檔把所有感測器 frameId 都寫成 base_link。

    最致命的是點雲：NDT 用 frame_id 查 odom→<frame> 來擺放點雲，
    標成 base_link 會讓雲落在離地 0.13 m，而地圖是感測器在 1.43 m 錄的。
    """
    f = spec.frames
    targets = (
        (f"{G_VELO}/ros2_publish_point_cloud", f.velodyne),
        (f"{G_IMU}/ros2_publish_imu", f.imu),
        (f"{G_2D}/publish_front_2d_lidar_scan", f.ydlidar_front),
        (f"{G_2D}/publish_back_2d_lidar_scan", f.ydlidar_back),
    )
    return [c for c in (
        _set_attr(stage, node, "inputs:frameId", frame, Sdf.ValueTypeNames.String)
        for node, frame in targets
    ) if c]


def measure_corridor_floor_top(stage: Usd.Stage) -> float:
    """量出走廊 mesh 地板上表面的世界 z。

    用**眾數**（1 cm bin 內頂點最多的那一層），不是中位數或最大值。
    實測該 mesh 在地板帶內有三層：

        z = -0.61  n=  89   板底
        z = -0.32  n=1902   ← 地板上表面（密度壓倒性）
        z = -0.17  n= 682   門檻／牆基之類

    中位數或「上半部中位數」會被 -0.17 那群拉走（實測算出 -0.1733，
    與執行期量到的 base_footprint -0.3229 差 15 cm）。
    """
    from collections import Counter

    mesh = UsdGeom.Mesh(_require(stage, CORRIDOR_MESH))
    m = UsdGeom.XformCache().GetLocalToWorldTransform(mesh.GetPrim())
    band = [m.Transform(pt)[2] for pt in mesh.GetPointsAttr().Get() if -0.70 < m.Transform(pt)[2] < 0.0]
    if not band:
        raise RuntimeError("找不到走廊地板面的頂點")
    hist = Counter(round(z, 2) for z in band)
    mode_z = hist.most_common(1)[0][0]
    # 回傳該層的精確平均，避免 1 cm bin 的量化誤差
    layer = [z for z in band if abs(z - mode_z) < 0.005]
    return sum(layer) / len(layer)


def align_navmesh_floor_to_corridor(stage: Usd.Stage, spec: S.SimRosSpec) -> list[Change]:
    """把 NavFloor 輔助板抬到與走廊地板同高，保留碰撞。

    為什麼不是關掉碰撞：2026-09-21 實測，關掉之後車從 c28 開約 10 m 到
    World(-17.3,+7.8) 就掉出世界（z 一路掉到 -7102 m），因為走廊 mesh
    在那裡沒有地板幾何 —— NavFloor 才是那片區域唯一的可行走面。

    抬高對齊同時解決兩件事：
      - 碰撞面覆蓋整個可行走區域（車不會掉下去）
      - 上表面與走廊地板重合 → 光達只看到一層地板，不再有假地板
    """
    floor_top = measure_corridor_floor_top(stage)
    changes: list[Change] = []
    for path in NAV_FLOOR_PRIMS:
        prim = _require(stage, path)
        half = float(prim.GetAttribute("size").Get() or 2.0) / 2.0
        scale_z = float(prim.GetAttribute("xformOp:scale").Get()[2])
        t = prim.GetAttribute("xformOp:translate").Get()
        desired_z = floor_top - half * scale_z          # 讓上表面剛好落在走廊地板
        if abs(t[2] - desired_z) < 1e-6:
            continue
        new_t = Gf.Vec3d(t[0], t[1], desired_z)
        prim.GetAttribute("xformOp:translate").Set(new_t)
        changes.append(Change(path, f"xformOp:translate（上表面對齊走廊地板 {floor_top:+.4f}）",
                              tuple(t), tuple(new_t)))
        # 碰撞必須保留 —— 這是那片區域唯一的可行走面
        c = _set_attr(stage, path, "physics:collisionEnabled", True, Sdf.ValueTypeNames.Bool)
        if c:
            changes.append(c)
    return changes


def disable_broken_2d_lidars(stage: Usd.Stage, spec: S.SimRosSpec) -> list[Change]:
    """停用前後 2D 光達 —— 它們本來就沒有資料，卻每幀在建 render product。"""
    if spec.enable_2d_lidars:
        return []
    return _disable_nodes(stage, TWO_D_LIDAR_NODES, "資產缺失、無資料且耗 GPU")


def retarget_point_cloud_topic(stage: Usd.Stage, spec: S.SimRosSpec) -> list[Change]:
    """啟用拖影注入時，Isaac 改發 /velodyne_points_ideal。

    與 odom 同樣的分層：Isaac 出理想值，lidar_motion_smear 注入真實世界的
    掃描運動畸變後才成為下游吃的 /velodyne_points。
    Isaac 的 PhysX Lidar 給的是單一瞬間快照，而車端判定「點雲缺逐點去畸變」
    是 NDT 的主要誤差源 —— 不注入的話模擬的 NDT 會穩到完全不像實車。
    """
    c = _set_attr(stage, f"{G_VELO}/ros2_publish_point_cloud", "inputs:topicName",
                  spec.isaac_point_cloud_topic, Sdf.ValueTypeNames.String)
    return [c] if c else []


def rewire_timestamps_to_simulation_time(stage: Usd.Stage) -> list[Change]:
    """把 timeStamp 從 OnPlaybackTick.outputs:time 改接 IsaacReadSimulationTime。"""
    changes: list[Change] = []
    for pub_node, time_node in STAMP_REWIRES:
        _require(stage, time_node)  # 確認那顆懸空節點確實存在
        prim = _require(stage, pub_node)
        attr = prim.GetAttribute("inputs:timeStamp")
        if not attr or not attr.IsValid():
            attr = prim.CreateAttribute("inputs:timeStamp", Sdf.ValueTypeNames.Double, custom=True)
        target = Sdf.Path(f"{time_node}.outputs:simulationTime")
        before = [str(p) for p in attr.GetConnections()]
        if before == [str(target)]:
            continue
        attr.SetConnections([target])
        changes.append(Change(pub_node, "inputs:timeStamp <-", before, [str(target)]))
    return changes


def fix_differential_drive(stage: Usd.Stage, spec: S.SimRosSpec) -> list[Change]:
    """原始檔 wheelDistance=0.7，既非實車 0.559212 也非 USD 自身輪距 0.554。"""
    d = spec.drive
    node = f"{G_DRIVE}/differential_controller"
    F = Sdf.ValueTypeNames.Double
    return [c for c in (
        _set_attr(stage, node, "inputs:wheelDistance", d.wheel_distance_m, F),
        _set_attr(stage, node, "inputs:wheelRadius", d.wheel_radius_m, F),
        _set_attr(stage, node, "inputs:maxLinearSpeed", d.max_linear_speed_m_s, F),
        _set_attr(stage, node, "inputs:maxAngularSpeed", d.max_angular_speed_rad_s, F),
        _set_attr(stage, node, "inputs:maxAcceleration", d.max_acceleration_m_s2, F),
        _set_attr(stage, node, "inputs:maxDeceleration", d.max_acceleration_m_s2, F),
    ) if c]


def fix_lidar_range(stage: Usd.Stage, spec: S.SimRosSpec) -> list[Change]:
    """原始檔 maxRange=20 m，不足以覆蓋 NDT 的 ±40 m crop。"""
    F = Sdf.ValueTypeNames.Float
    return [c for c in (
        _set_attr(stage, P_LIDAR, "maxRange", float(spec.lidar.max_range_m), F),
        _set_attr(stage, P_LIDAR, "minRange", float(spec.lidar.min_range_m), F),
    ) if c]


def align_sensor_prims_to_urdf(stage: Usd.Stage, spec: S.SimRosSpec) -> list[Change]:
    """把 velodyne prim 的實體位置搬到與實車 URDF 一致。

    方案 A 下 TF 由 robot_state_publisher 依 URDF 發，從 odom 的參考點
    base_footprint 算到 velodyne_link 是 (-0.02, 0, 1.43)。若模擬的感測器實際
    裝在別的位置，TF 說的與點雲實際來源就不一致，等於在 NDT 注入系統性誤差。
    原始檔實測 base_footprint→velodyne = 1.4724 m，差 42.4 mm。

    ⚠ 只改平移，不動旋轉。velodyne prim 帶 180° Z 旋轉，其下的 Lidar prim
       也帶 180° Z 旋轉，兩者相消後等效姿態與 base_link 一致（對應 URDF 的
       rpy="0 0 0"）。動旋轉會讓點雲繞 Z 轉 180°，NDT 直接失效。
    """
    anchor = _require(stage, P_BASE_FOOTPRINT)
    velo = _require(stage, P_VELODYNE)

    base_xf = UsdGeom.Xformable(anchor).GetLocalTransformation()
    offset = Gf.Vec3d(*spec_offset(spec))
    # ⚠ pxr 是 row-vector 慣例（p' = p * M）。不可寫成 rotMatrix * offset ——
    #    那是 column-vector 乘法，等同乘上轉置（反向旋轉），x 會差 3.6 cm。
    #    正解是矩陣合成：velodyne_to_parent = velodyne_to_base_link * base_link_to_parent
    velo_to_base = Gf.Matrix4d(1.0).SetTranslate(offset)
    desired = (velo_to_base * base_xf).ExtractTranslation()

    attr = velo.GetAttribute("xformOp:translate")
    before = attr.Get()
    if before is not None and Gf.IsClose(before, desired, 1e-9):
        return []
    attr.Set(desired)
    return [Change(P_VELODYNE, "xformOp:translate", tuple(before), tuple(desired))]


def measure_base_footprint_to_velodyne(stage: Usd.Stage) -> Gf.Vec3d:
    """實際量測 stage 上 base_footprint→velodyne 的平移，單位公尺。

    為什麼錨定 base_footprint 而不是 base_link：
    IsaacComputeOdometry 的 chassisPrim 指向 base_footprint prim，因此 TF 樹裡
    ``odom→base_footprint`` 對應的實體就是那顆 prim。robot_state_publisher 之後
    宣稱的 base_footprint→base_link→velodyne_link 是 URDF 的 0.13+1.3=1.43，
    所以模擬中光達相對 **base_footprint prim** 的實距必須恰好等於 1.43。

    （USD 自身的 base_footprint→base_link 是 0.1336 而非 URDF 的 0.13，
    若改錨定 base_link 會殘留 3.66 mm 的系統性誤差。）

    這是驗收量：驗結果而不是驗算式，才擋得住 row-vector / column-vector
    乘錯方向這類錯誤。
    """
    cache = UsdGeom.XformCache()
    fp_w = cache.GetLocalToWorldTransform(_require(stage, P_BASE_FOOTPRINT))
    velo_w = cache.GetLocalToWorldTransform(_require(stage, P_VELODYNE))
    return (velo_w * fp_w.GetInverse()).ExtractTranslation()


def spec_offset(spec: S.SimRosSpec) -> tuple[float, float, float]:
    """base_footprint→velodyne_link 的 URDF 偏移。

    URDF 兩段關節的 rpy 都是 "0 0 0"，所以直接相加即可：
      base_footprint →(0, 0, 0.13)→ base_link →(-0.02, 0, 1.3)→ velodyne_link
    """
    a = S.BASE_FOOTPRINT_TO_BASE_LINK_M
    b = S.BASE_LINK_TO_VELODYNE_M
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def measure_base_footprint_world_pose(stage: Usd.Stage) -> tuple[float, float, float, float]:
    """回傳 base_footprint 的 World 位姿 (x, y, z, yaw)。驗收用。"""
    import math
    m = UsdGeom.XformCache().GetLocalToWorldTransform(_require(stage, P_BASE_FOOTPRINT))
    t = m.ExtractTranslation()
    return (t[0], t[1], t[2], math.atan2(m[0][1], m[0][0]))


def spawn_robot_at_routing_node(stage: Usd.Stage, spec: S.SimRosSpec,
                                csv_path: Path | None = None,
                                node_name: str | None = None) -> list[Change]:
    """把機器人搬到指定 routing 節點（預設 c28）。

    搬的是最外層的 ``/World/charger_rover4_5_0`` Xform —— 它底下的
    ``charger_rover_urdf5`` 帶 PhysicsArticulationRootAPI、``base_footprint``
    帶 PhysicsRigidBodyAPI，直接動 articulation 內的 prim 會破壞物理設定。

    只改 x / y / yaw，**保留 z 與 roll / pitch**，因為現況的 base_footprint
    World z = −0.3208 已經貼在 USD 地板（−0.325，差 4 mm）上。
    """
    import math

    node_name = node_name or spec_spawn_node(spec)
    csv_path = csv_path or (Path(__file__).resolve().parents[1] / S.ROUTING_STATION_JSON)
    nodes = S.read_station_nodes(csv_path)
    if node_name not in nodes:
        raise LookupError(f"{Path(csv_path).name} 沒有站 {node_name}")
    mx, my, _ = nodes[node_name]
    # JSON 的 rooms 沒有朝向（rw=1, rz=0），用「面向走廊下一站」決定 yaw，
    # 否則車會朝著牆生出來。
    face = S.SPAWN_FACING_NODE
    if face in nodes and face != node_name:
        fx, fy, _ = nodes[face]
        myaw = math.atan2(fy - my, fx - mx)
    else:
        myaw = 0.0
    wx, wy, wyaw = S.map_to_world(mx, my, myaw)

    cache = UsdGeom.XformCache()
    root = _require(stage, P_ROBOT_ROOT)
    T_root = cache.GetLocalToWorldTransform(root)
    T_fp = cache.GetLocalToWorldTransform(_require(stage, P_BASE_FOOTPRINT))

    cur = measure_base_footprint_world_pose(stage)
    if (abs(cur[0] - wx) < 1e-6 and abs(cur[1] - wy) < 1e-6
            and abs(((cur[3] - wyaw + math.pi) % (2 * math.pi)) - math.pi) < 1e-9):
        return []

    # base_footprint 相對於 root（row-vector: T_fp = A * T_root）
    A = T_fp * T_root.GetInverse()

    # 目標：只改 x/y/yaw，z 與 roll/pitch 沿用現況
    delta_yaw = wyaw - cur[3]
    Rz = Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(delta_yaw)))
    T_fp_new = Gf.Matrix4d(T_fp)
    T_fp_new.SetTranslateOnly(Gf.Vec3d(0, 0, 0))
    T_fp_new = T_fp_new * Rz
    T_fp_new.SetTranslateOnly(Gf.Vec3d(wx, wy, cur[2]))

    T_root_new = A.GetInverse() * T_fp_new
    q = T_root_new.ExtractRotationQuat()
    t_attr = root.GetAttribute("xformOp:translate")
    o_attr = root.GetAttribute("xformOp:orient")
    before_t = t_attr.Get()
    # xformOp 的精度依 prim 而異（這顆 root 是 quatf / double3），要照實際型別寫
    trans = T_root_new.ExtractTranslation()
    t_attr.Set(Gf.Vec3f(trans) if "float3" in str(t_attr.GetTypeName()).lower() else trans)
    quat_cls = Gf.Quatf if "quatf" in str(o_attr.GetTypeName()).lower() else Gf.Quatd
    o_attr.Set(quat_cls(q.GetReal(), *q.GetImaginary()))

    return [Change(P_ROBOT_ROOT, f"spawn @ routing '{node_name}'",
                   f"base_footprint ({cur[0]:.3f}, {cur[1]:.3f}, yaw {math.degrees(cur[3]):.2f}deg)",
                   f"map ({mx:.2f}, {my:.2f}) = World ({wx:.3f}, {wy:.3f}, yaw {math.degrees(wyaw) % 360:.2f}deg)"),
            Change(P_ROBOT_ROOT, "xformOp:translate", tuple(before_t), tuple(T_root_new.ExtractTranslation()))]


def spec_spawn_node(spec: S.SimRosSpec) -> str:
    return S.SPAWN_ROUTING_NODE


def align_sensor_joint_to_urdf(stage: Usd.Stage, spec: S.SimRosSpec) -> list[Change]:
    """把 velodyne 的**物理關節**對齊 URDF —— 這才是執行期真正生效的定義。

    ⚠ 2026-09-21 實測的教訓：``velodyne`` 帶 PhysicsRigidBodyAPI，
    物理開跑後它的位置由 ``velodyne_base_mount_joint`` 的 localPos 決定，
    **改 xformOp:translate 會被 PhysX 蓋掉**。Xform 只影響初始／視覺姿態。

    方案 A 下 robot_state_publisher 會依 URDF 宣稱
    base_footprint→velodyne_link = (-0.02, 0, 1.43)。物理必須一致，
    否則 TF 說的位置與點雲實際來源不符，等於直接餵錯誤給 NDT。

    關節鏈（皆以 base_link 為 body0）：
        base_base_footprint.localPos0.z = -0.13366   → base_footprint 在 base_link 下方
        velodyne_base_mount_joint.localPos0          → velodyne 在 base_link 上的位置
    所以 velodyne 關節該設為 URDF 總量 + base_link→base_footprint 的偏移。
    """
    fp_joint = _require(stage, P_FOOTPRINT_JOINT)
    fp_off = fp_joint.GetAttribute("physics:localPos0").Get()   # base_link → base_footprint
    target = spec_offset(spec)                                   # base_footprint → velodyne (URDF)
    desired = Gf.Vec3f(float(target[0] + fp_off[0]),
                       float(target[1] + fp_off[1]),
                       float(target[2] + fp_off[2]))
    attr = _require(stage, P_VELO_JOINT).GetAttribute("physics:localPos0")
    before = attr.Get()
    if before is not None and Gf.IsClose(Gf.Vec3f(before), desired, 1e-7):
        return []
    attr.Set(desired)
    return [Change(P_VELO_JOINT, "physics:localPos0 (執行期生效的定義)",
                   tuple(before), tuple(desired))]



def place_corridor_obstacles(stage: Usd.Stage, spec: S.SimRosSpec) -> list[Change]:
    """在走廊放帶碰撞體的障礙物，讓 PhysX 光達打得到、車必須閃避。

    ⚠ 不用 omni.anim.people 的角色：論文 §2.8.2 已載明「型別為 Lidar 的 RTX 光達
    不對骨架網格角色做光線追蹤」，而本專案用的 PhysX 光達是對**碰撞體** raycast，
    骨架角色沒有碰撞體 → 同樣照不到。且 People 資產是抓不到的 S3 連結。
    用帶碰撞體的幾何代理才是能被感知、能被閃避的障礙物。

    位置用 map frame 指定（與 routing 站同一個座標系），底部貼在走廊地板上。
    """
    if not spec.obstacles:
        return []
    from pxr import UsdPhysics

    floor_top = measure_corridor_floor_top(stage)
    root = stage.GetPrimAtPath(OBSTACLE_ROOT)
    if not root or not root.IsValid():
        root = UsdGeom.Xform.Define(stage, OBSTACLE_ROOT).GetPrim()

    changes: list[Change] = []
    for o in spec.obstacles:
        path = f"{OBSTACLE_ROOT}/{o.name}"
        if stage.GetPrimAtPath(path).IsValid():
            continue
        wx, wy, _ = S.map_to_world(o.map_x, o.map_y, 0.0)
        cz = floor_top + o.height / 2.0          # 底部貼地

        if o.kind == "person":
            g = UsdGeom.Cylinder.Define(stage, path)
            g.CreateRadiusAttr(float(o.radius))
            g.CreateHeightAttr(float(o.height))
            g.CreateAxisAttr("Z")
            g.CreateExtentAttr([(-o.radius, -o.radius, -o.height/2),
                                (o.radius, o.radius, o.height/2)])
            scale = Gf.Vec3d(1.0, 1.0, 1.0)
            desc = f"圓柱 r={o.radius} h={o.height}"
        else:
            g = UsdGeom.Cube.Define(stage, path)
            g.CreateSizeAttr(1.0)
            g.CreateExtentAttr([(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)])
            scale = Gf.Vec3d(float(o.size_x), float(o.size_y), float(o.height))
            desc = f"方箱 {o.size_x}x{o.size_y}x{o.height}"

        # ⚠ 用單一 transform 矩陣，不要 AddScaleOp + AddTranslateOp —— 實測那樣
        #    平移會被縮放乘到（cart 底部差 12.7 mm = 0.1267 x 0.9），測試才抓到。
        m = Gf.Matrix4d(1.0)
        m.SetScale(scale)
        m = m * Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(wx, wy, cz))
        g.MakeMatrixXform().Set(m)
        # 靜態碰撞體：PhysX 光達 raycast 打得到，車也撞得到。
        # 不加 RigidBodyAPI —— 這些是固定障礙，不需要被物理推動。
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        changes.append(Change(path, f"{o.kind} @ map({o.map_x:+.1f},{o.map_y:+.1f})",
                              None, f"World({wx:.3f},{wy:.3f},{cz:.3f}) {desc}"))
    return changes


def add_clock_publisher(stage: Usd.Stage, spec: S.SimRosSpec) -> list[Change]:
    """新增 /clock 發佈 graph。

    原始檔 5 張 graph 裡完全沒有 ROS2PublishClock。沒有 /clock，
    交接單 §4 要求的「整條鏈 use_sim_time:=true」就做不到。
    """
    if not spec.publish_clock:
        return []
    if stage.GetPrimAtPath(CLOCK_GRAPH).IsValid():
        return []

    graph = stage.DefinePrim(CLOCK_GRAPH, "OmniGraph")
    graph.CreateAttribute("evaluator:type", Sdf.ValueTypeNames.Token, custom=True).Set("execution")
    graph.CreateAttribute("evaluationMode", Sdf.ValueTypeNames.Token, custom=True).Set("Automatic")
    graph.CreateAttribute("fabricCacheBacking", Sdf.ValueTypeNames.Token, custom=True).Set("StageWithoutHistory")
    graph.CreateAttribute("fileFormatVersion", Sdf.ValueTypeNames.Int2, custom=True).Set(Gf.Vec2i(1, 9))
    graph.CreateAttribute("pipelineStage", Sdf.ValueTypeNames.Token, custom=True).Set("pipelineStageSimulation")

    def node(name: str, node_type: str) -> Usd.Prim:
        p = stage.DefinePrim(f"{CLOCK_GRAPH}/{name}", "OmniGraphNode")
        p.CreateAttribute("node:type", Sdf.ValueTypeNames.Token, custom=True).Set(node_type)
        p.CreateAttribute("node:typeVersion", Sdf.ValueTypeNames.Int, custom=True).Set(1)
        return p

    tick = node("on_playback_tick", "omni.graph.action.OnPlaybackTick")
    sim_time = node("isaac_read_simulation_time", "isaacsim.core.nodes.IsaacReadSimulationTime")
    pub = node("ros2_publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock")

    pub.CreateAttribute("inputs:topicName", Sdf.ValueTypeNames.String, custom=True).Set(
        spec.topics.clock.lstrip("/")
    )
    pub.CreateAttribute("inputs:execIn", Sdf.ValueTypeNames.UInt, custom=True).SetConnections(
        [Sdf.Path(f"{tick.GetPath()}.outputs:tick")]
    )
    pub.CreateAttribute("inputs:timeStamp", Sdf.ValueTypeNames.Double, custom=True).SetConnections(
        [Sdf.Path(f"{sim_time.GetPath()}.outputs:simulationTime")]
    )
    return [Change(CLOCK_GRAPH, "graph", None, "created ROS2PublishClock")]


# --------------------------------------------------------------------------
# 組裝
# --------------------------------------------------------------------------




def place_map_patch(stage: Usd.Stage, spec: S.SimRosSpec) -> list[Change]:
    """把「地圖有、USD 沒有」的天花板結構補回場景（隱形碰撞體）。

    走廊是兩道平行長牆；牆面光禿禿就沒有**沿走廊方向**的特徵 —— 前進一
    公尺與原地不動看起來幾乎一樣，NDT 估不出縱向位移（孔徑問題）。實測
    導航失敗時「車走了 4.7 m，NDT 只認為移動 0.5 m」正是這個徵狀。

    實測走廊段有 25.5% 的地圖結構 USD 完全沒有，96% 集中在 1.3~2.3 m。

    幾何直接從 NDT 地圖體素化而來（見 map_patch 模組的說明與限制）。

    ⚠ 設成 **invisible**：``visibility`` 是純渲染屬性，PhysX 的碰撞查詢走
    另一條路徑，所以光達照樣打得到。這樣 Isaac GUI 與 RViz 都不會被天花板
    的方塊擋住視線（RViz 的**即時點雲**仍會出現打到它們的回波 —— 那是應該
    的，真實世界那裡確實有東西）。

    ⚠ 靜態碰撞體（不加 RigidBodyAPI）：這些結構不會動，而且 kinematic 剛體
    是無限質量，萬一與車重疊會把車彈飛（見 character_colliders 的教訓）。
    """
    if not spec.map_patch:
        return []
    import numpy as np
    from pxr import UsdPhysics, Vt

    from compare_map_vs_usd import read_pcd, sample_usd_surfaces, world_to_map
    from map_patch import (CEILING_BAND, MISSING_TOL_M, VOXEL_M, boxes_to_mesh,
                           voxel_centers, voxelize_missing)

    pcd = Path(spec.map_patch_pcd)
    if not pcd.exists():
        print(f"[build] ⚠ 找不到地圖 {pcd}，略過補丁")
        return []

    M = read_pcd(pcd)
    U = sample_usd_surfaces(str(spec.map_patch_source_usd), "/World/Env_0", 600000)
    Umap = np.column_stack([world_to_map(U[:, :2]),
                            U[:, 2] + S.WORLD_TO_MAP_Z_OFFSET])

    x0, x1, y0, y1 = spec.map_patch_bounds
    zl, zh = CEILING_BAND

    def crop(P):
        m = ((P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1)
             & (P[:, 2] >= zl) & (P[:, 2] < zh))
        return P[m]

    cells = voxelize_missing(crop(M), crop(Umap), VOXEL_M, MISSING_TOL_M)
    if len(cells) == 0:
        return []
    centers_map = voxel_centers(cells, VOXEL_M)

    # map → world（反向變換）：先扣平移、再轉 -yaw
    yaw = S.WORLD_TO_MAP_YAW_RAD
    tx, ty = S.WORLD_TO_MAP_TRANSLATION
    c_, s_ = math.cos(yaw), math.sin(yaw)
    dx, dy = centers_map[:, 0] - tx, centers_map[:, 1] - ty
    centers_world = np.column_stack([
        dx * c_ + dy * s_,
        -dx * s_ + dy * c_,
        centers_map[:, 2] - S.WORLD_TO_MAP_Z_OFFSET,
    ])

    pts, counts, idx = boxes_to_mesh(centers_world, VOXEL_M)
    mesh = UsdGeom.Mesh.Define(stage, MAP_PATCH_PATH)
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    a = np.array(pts)
    mesh.CreateExtentAttr([tuple(a.min(axis=0)), tuple(a.max(axis=0))])
    UsdGeom.Imageable(mesh.GetPrim()).CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    mc = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
    mc.CreateApproximationAttr().Set("none")      # 靜態三角網格，BVH 查詢

    return [Change(MAP_PATCH_PATH, "地圖補丁（天花板層）", None,
                   f"{len(cells)} 方塊 / {len(counts)} 三角面  "
                   f"體素 {VOXEL_M} m  z∈[{zl},{zh})  隱形＋靜態碰撞體")]


STEPS = (
    ("停用 Isaac 的 TF 發佈（方案 A）", disable_isaac_tf_publishers),
    ("odom topic 改 /odom_gt 讓 injector 插入", retarget_odometry_topic),
    ("修正感測器 frameId", fix_sensor_frame_ids),
    ("點雲改發理想 topic 供拖影注入", retarget_point_cloud_topic),
    ("光達量程覆蓋 NDT crop", fix_lidar_range),
    ("差速控制器參數對齊實車", fix_differential_drive),
    ("velodyne 實體位置對齊 URDF（Xform）", align_sensor_prims_to_urdf),
    ("velodyne 物理關節對齊 URDF（執行期生效）", align_sensor_joint_to_urdf),
    ("NavFloor 抬高對齊走廊地板", align_navmesh_floor_to_corridor),
    ("停用無資料的 2D 光達", disable_broken_2d_lidars),
    ("走廊障礙物", place_corridor_obstacles),
    ("地圖補丁（天花板層）", place_map_patch),
    ("新增 /clock 發佈", add_clock_publisher),
    ("機器人生成於 routing 站 c28（JSON 站表）", spawn_robot_at_routing_node),
)


def apply_spec(stage: Usd.Stage, spec: S.SimRosSpec) -> list[tuple[str, list[Change]]]:
    report = [("時戳改接模擬時間", rewire_timestamps_to_simulation_time(stage))]
    for label, fn in STEPS:
        report.append((label, fn(stage, spec)))
    return sorted(report, key=lambda x: STEPS_ORDER.get(x[0], 99))


STEPS_ORDER = {label: i for i, (label, _) in enumerate(STEPS)}
STEPS_ORDER["時戳改接模擬時間"] = -1


def build(input_usd: Path, output_usda: Path, spec: S.SimRosSpec) -> list[tuple[str, list[Change]]]:
    """建立 override layer 並套用規格。"""
    output_usda.parent.mkdir(parents=True, exist_ok=True)
    if output_usda.exists():
        output_usda.unlink()

    layer = Sdf.Layer.CreateNew(str(output_usda))
    layer.subLayerPaths.append(str(input_usd.resolve()))
    stage = Usd.Stage.Open(layer)
    stage.SetEditTarget(Usd.EditTarget(layer))

    report = apply_spec(stage, spec)
    layer.Save()
    return report


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", type=Path,
                    default=Path("/home/aa/Ros/charge_rl/assets/3F/3floor_ver_1.usd"))
    ap.add_argument("--obstacles", action="store_true",
                    help="在走廊放帶碰撞體的障礙物（人體圓柱代理＋推車方箱）。"
                         "不帶此旗標＝淨空走廊，做定位基準時用。")
    ap.add_argument("--map-patch", action="store_true",
                    help="把「地圖有、USD 沒有」的天花板結構補成隱形碰撞體，"
                         "給 NDT 提供沿走廊方向的特徵（解孔徑問題）。")
    ap.add_argument("--output", type=Path,
                    default=Path(__file__).resolve().parents[1] / "assets" / "3floor_ver_1_ros_fixed.usda")
    args = ap.parse_args(argv)

    if not args.input.exists():
        print(f"[ERROR] 找不到輸入 USD: {args.input}", file=sys.stderr)
        return 2

    spec = S.SimRosSpec().with_tf_ownership(S.default_tf_ownership())
    if args.obstacles:
        spec = replace(spec, obstacles=S.DEFAULT_OBSTACLES)
    if args.map_patch:
        spec = replace(spec, map_patch=True)
    report = build(args.input, args.output, spec)

    total = 0
    for label, changes in report:
        mark = "·" if not changes else "✔"
        print(f"{mark} {label}  ({len(changes)} 項)")
        for c in changes:
            print(c)
        total += len(changes)
    print(f"\n共 {total} 項修改，已寫入 {args.output}")
    print(f"原始 USD 未被修改：{args.input}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
