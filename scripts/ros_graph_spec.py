"""模擬端 ROS 2 介面規格（純資料層，無 Isaac 相依）。

這支模組是「模擬要長得跟實車一樣」的單一事實來源(single source of truth)。
所有數字都標註了車端出處，改動前請先回去核對該檔案。

與 build_ros_graph.py 的分工：
  - 本檔：描述「目標狀態該是什麼」，可在沒有 Isaac Sim 的環境下 import 與測試
  - build_ros_graph.py：把本檔的規格套用到 USD stage（需要 Isaac Sim runtime）

車端對照文件：~/rover_rl/docs/2026-09-21_IsaacLab模擬NDT與Routing移植_給PC端.md
PC 端對照文件：docs/2026-09-21_模擬ROS契約對照_PC端回覆.md
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Mapping

# --------------------------------------------------------------------------
# 1. 實車硬體真值 —— 出處逐項標註，不可憑記憶改
# --------------------------------------------------------------------------

#: campusrover_base/campusrover_description/urdf/campusrover_chgh.xacro:115-117
BASE_FOOTPRINT_TO_BASE_LINK_M: tuple[float, float, float] = (0.0, 0.0, 0.13)

#: campusrover_chgh.xacro:132-135
BASE_LINK_TO_VELODYNE_M: tuple[float, float, float] = (-0.02, 0.0, 1.3)

#: driver_chgh.yaml: wheel_base_length
WHEEL_BASE_LENGTH_M: float = 0.559212

#: driver_chgh.yaml: (left_wheel_diameter + right_wheel_diameter) / 4
WHEEL_RADIUS_M: float = (0.244211 + 0.239577) / 4.0

#: driver_chgh.yaml: profile_omega_max
MAX_ANGULAR_SPEED_RAD_S: float = 1.2

#: driver_chgh.yaml: max_speed
MAX_LINEAR_SPEED_M_S: float = 1.5

#: driver_chgh.yaml: acc_max
MAX_ACCELERATION_M_S2: float = 1.2

#: driver_chgh.yaml: publish_rate（odom 發佈頻率）
ODOM_RATE_HZ: float = 20.0

#: velodyne_points.yaml: rpm 600 → 10 Hz
LIDAR_SCAN_RATE_HZ: float = 10.0

#: NDT voxel_grid_filter 的 crop 範圍（交接單 §3）。超出此距離的點 NDT 根本不用，
#: 但低於此距離會讓走廊遠端結構消失 —— 原 USD 設 20 m 太短。
NDT_CROP_RANGE_M: float = 40.0


# --------------------------------------------------------------------------
# 2. Frame 名稱 —— 全部取自車端實際設定，不是猜的
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class FrameNames:
    """實車 TF frame 命名。出處見各欄位註解。"""

    map: str = "map"
    odom: str = "odom"
    #: driver_chgh.yaml: ros.base_frame
    base_footprint: str = "base_footprint"
    base_link: str = "base_link"
    #: campusrover_sensors/launch/config/velodyne_points.yaml:10
    velodyne: str = "velodyne_link"
    #: campusrover_chgh.xacro imu_link_joint
    imu: str = "imu_link"
    #: ydlidar_front.yaml:4
    ydlidar_front: str = "ydlidar_front_link"
    #: ydlidar_back.yaml:4
    ydlidar_back: str = "ydlidar_back_link"


FRAMES = FrameNames()


# --------------------------------------------------------------------------
# 3. Topic 名稱
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class TopicNames:
    """模擬端要發/收的 topic。

    注意 ``odom`` 發的是 ``/odom_gt`` 而非 ``/odom``：
    odom_drift_injector 吃 ``/odom_gt`` → 劣化 → 吐 ``/odom``。
    若模擬直接佔用 ``/odom``，injector 無法插入（交接單 §5.4）。
    """

    clock: str = "/clock"
    #: NDT 與 policy 實際訂閱的點雲。由 lidar_motion_smear 節點產生。
    point_cloud: str = "/velodyne_points"
    #: Isaac 直接輸出的理想點雲（瞬間快照，無運動拖影）。
    #: 與 odom 同樣的分層：模擬發理想值，降級節點插在中間注入真實世界的缺陷。
    point_cloud_ideal: str = "/velodyne_points_ideal"
    odom_ground_truth: str = "/odom_gt"
    imu: str = "/imu"
    scan_front: str = "/scan"
    scan_back: str = "/back_scan"
    #: policy 發 /input/nav_cmd_vel，經 lcr_cmd_vel_mux 才成 /cmd_vel
    cmd_vel: str = "/cmd_vel"
    tf: str = "/tf"


TOPICS = TopicNames()


# --------------------------------------------------------------------------
# 4. TF 發佈權責（TF ownership）
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class TfOwnership:
    """哪一邊負責發哪一段 TF。

    ROS 的 TF 樹要求每個 frame 只能有**一個** parent。同一段邊若被兩個節點發，
    tf2 會交替接受兩份資料，查詢結果隨機跳動，且不會報錯 —— 極難 debug。
    所以這裡必須明確切割，不能「都發、反正一樣」。

    模擬場上有三個可能的發佈者：
      1. Isaac Action Graph          —— 數字來自 USD 幾何
      2. odom_drift_injector         —— publish_tf 參數，發 odom→base_footprint
      3. robot_state_publisher       —— 數字來自實車 URDF campusrover_chgh.xacro
    """

    #: Isaac 是否發 odom→base_footprint。
    isaac_publishes_odom_tf: bool = False
    #: Isaac 是否發 base_link→{velodyne, imu, ydlidar} 等感測器 static TF。
    isaac_publishes_sensor_tf: bool = False
    #: Isaac 是否發 base_footprint→base_link。
    isaac_publishes_footprint_tf: bool = False


def default_tf_ownership() -> TfOwnership:
    """回傳建議的 TF 權責切割。

    TODO(user): 由你決定這三個旗標。理由寫在下方，但這是架構決策，該你拍板。

    我的建議是 **三個全部 False**，也就是 Isaac 一條 TF 都不發：

      map            → odom            由 ndt_localizer 發（與實車同一支程式）
      odom           → base_footprint  由 odom_drift_injector 發（publish_tf:=true）
      base_footprint → base_link       由 robot_state_publisher 依 URDF 發
      base_link      → velodyne_link   同上

    這樣切的好處：
      - TF 的**數字**全部來自實車 URDF，與實車逐位元一致，不受 USD 幾何誤差污染
        （USD 的 base_link→velodyne 是 1.3388 m，實車 URDF 是 1.3 m，差 38.8 mm）
      - odom 的漂移由 injector 統一注入，位置與 TF 不會各自劣化、互相矛盾
      - Isaac 只負責「感測器資料」與「真值 odom topic」，職責單純

    代價與必須配套的動作：
      - 模擬裡感測器 prim 的**實體位置**必須被移到與 URDF 一致，否則 TF 說 1.43 m、
        點雲實際來自 1.4724 m，會在 NDT 注入 42.4 mm 的系統性誤差。
        （build_ros_graph.py 的 align_sensor_prims_to_urdf() 負責這件事）
      - 必須確保 robot_state_publisher 有被 launch —— 它不在 deploy_full.launch.py 裡，
        而在 rover2_ws 的 campusrover_demo_launch.py

    如果你想讓模擬自給自足、不依賴 rover2_ws 的 robot_state_publisher，
    那就把 isaac_publishes_sensor_tf 與 isaac_publishes_footprint_tf 設 True，
    但務必同時執行 align_sensor_prims_to_urdf()，否則 TF 會帶著 USD 的幾何誤差。
    決策（2026-09-21，方案 A）：三個旗標全部 False。
    Isaac 只負責「感測器資料」與「真值 odom topic」，一條 TF 都不發。
    理由是目標為「模擬跑實車同一套節點」，TF 數字直接取自實車 URDF 最忠實，
    論文上也可主張模擬與實車共用同一份運動學描述。
    """
    return TfOwnership(
        isaac_publishes_odom_tf=False,       # 交給 odom_drift_injector (publish_tf:=true)
        isaac_publishes_sensor_tf=False,     # 交給 robot_state_publisher (campusrover_chgh.xacro)
        isaac_publishes_footprint_tf=False,  # 同上
    )


# --------------------------------------------------------------------------
# 5. 感測器規格
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class LidarSpec:
    """VLP-16 規格。horizontal/vertical 欄位已與原 USD 一致，不需改。"""

    horizontal_fov_deg: float = 360.0
    horizontal_resolution_deg: float = 0.2      # → 1800 pts/ring
    vertical_fov_deg: float = 30.0              # ±15°
    vertical_resolution_deg: float = 2.0        # → 16 rings
    rotation_rate_hz: float = LIDAR_SCAN_RATE_HZ
    #: 原 USD 為 0.5。與訓練端 r_min 的差異見 CLAUDE.md v3 段落。
    min_range_m: float = 0.5
    #: 原 USD 為 20.0，不足以覆蓋 NDT 的 ±40 m crop。
    max_range_m: float = NDT_CROP_RANGE_M

    @property
    def num_rings(self) -> int:
        return int(round(self.vertical_fov_deg / self.vertical_resolution_deg)) + 1

    @property
    def points_per_ring(self) -> int:
        return int(round(self.horizontal_fov_deg / self.horizontal_resolution_deg))


@dataclass(frozen=True)
class DifferentialDriveSpec:
    """差速控制器參數。

    原 USD 的 wheelDistance=0.7 既不等於實車 0.559212，也不等於 USD 自身的
    輪關節間距 0.554（left/right_wheel_joint localPos0 的 y = ±0.27700004）。
    這會讓指令 ω 被放大約 1.26 倍。
    """

    wheel_distance_m: float = WHEEL_BASE_LENGTH_M
    wheel_radius_m: float = WHEEL_RADIUS_M
    max_linear_speed_m_s: float = MAX_LINEAR_SPEED_M_S
    max_angular_speed_rad_s: float = MAX_ANGULAR_SPEED_RAD_S
    max_acceleration_m_s2: float = MAX_ACCELERATION_M_S2


# --------------------------------------------------------------------------
# 6. 完整規格
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class SimRosSpec:
    """模擬端 ROS 介面的完整目標狀態。"""

    frames: FrameNames = FRAMES
    topics: TopicNames = TOPICS
    lidar: LidarSpec = field(default_factory=LidarSpec)
    drive: DifferentialDriveSpec = field(default_factory=DifferentialDriveSpec)
    tf_ownership: TfOwnership = field(default_factory=TfOwnership)
    #: 全鏈 use_sim_time 的前提：Isaac 必須發 /clock，且所有 publisher 的
    #: timeStamp 必須接 IsaacReadSimulationTime，不能接 OnPlaybackTick.time。
    publish_clock: bool = True
    use_simulation_time_for_stamps: bool = True
    #: 是否注入掃描運動拖影（車端判定的 NDT 主要誤差源）。
    #: True 時 Isaac 發 point_cloud_ideal，由 lidar_motion_smear 產生 point_cloud。
    inject_lidar_motion_smear: bool = True
    #: 前後 2D 光達（RPLIDAR S2E）。預設關閉：2026-09-21 實測 /scan 與 /back_scan
    #: 完全沒有資料（RPLidar_S2E 資產是抓不到的 S3 連結），卻每幀在建 render product。
    #: RL policy 只吃 /velodyne_points。要跑 costmap / MOT 才需要開。
    enable_2d_lidars: bool = False
    #: 走廊障礙物。空 tuple = 淨空走廊（做定位基準時用）。
    obstacles: tuple = ()
    #: 移動障礙物（行人）。空 tuple = 只有靜態障礙。
    moving_obstacles: tuple = ()

    @property
    def isaac_point_cloud_topic(self) -> str:
        """Isaac Action Graph 實際要發的點雲 topic。"""
        return self.topics.point_cloud_ideal if self.inject_lidar_motion_smear else self.topics.point_cloud

    def with_tf_ownership(self, ownership: TfOwnership) -> "SimRosSpec":
        """回傳套用新 TF 權責的副本（不可變更新）。"""
        return replace(self, tf_ownership=ownership)


def expected_static_tf_edges(spec: SimRosSpec) -> Mapping[str, str]:
    """回傳 child→parent 的 static TF 對照，用於驗證 TF 樹沒有雙 parent。"""
    f = spec.frames
    return {
        f.base_link: f.base_footprint,
        f.velodyne: f.base_link,
        f.imu: f.base_link,
        f.ydlidar_front: f.base_link,
        f.ydlidar_back: f.base_link,
    }


# --------------------------------------------------------------------------
# 7. World ↔ map 配準（2026-09-21 量測）
# --------------------------------------------------------------------------

#: USD World frame → NDT map frame 的 2D 剛體變換。
#:
#: 求法：把 USD 走廊 mesh 依面積取樣成 72 萬點，與 3F_314.pcd 各取「地板以上
#: 0.3–2.2 m 的牆面帶」投影成 2D 佔據影像，掃 yaw 0–360° 配合 FFT 互相關求平移。
#: 驗收量是「落在 PCD 佔據格上的 USD 點比例」而非原始相關值 —— 因為 USD 走廊
#: 只是 PCD 範圍的子集，原始相關值會偏好重疊面積大而非貼合得好。
#:
#: 結果：yaw=+96.80°，重疊率 0.96（res 0.25 m）/ 0.86（res 0.10 m）。
#: 疊圖確認紅色 USD 牆面精準貼合灰色真實點雲的南北主廊與東西支廊。
WORLD_TO_MAP_YAW_RAD: float = 1.6895509899999999  # = radians(96.80)
WORLD_TO_MAP_TRANSLATION: tuple[float, float] = (2.884359, 6.134030)
#: 地板高度差（World z + 此值 = map z）
WORLD_TO_MAP_Z_OFFSET: float = 0.1000

#: ⭐ **導航堆疊實際使用的站名表**。
#:
#: 2026-09-21 從 RViz 的 routes_visualization 綠色圖確認：畫出來的節點
#: （c1,c2,c4,c5,c6,c8,c9,c10,c12,c13,c14,c24,c25,c26,c29,c33,c37…）
#: 正是這支 JSON 的 29 個 room，不是 3F_info.csv 的 122 個節點。
#: mapinfo_db_handler 服務 /get_route_info、routing_to_path 解析站名，都用它。
#:
#: ⚠ 兩套表**同名但座標完全不同**（c28 差 8.54 m、c24 差 20.05 m、c3 差 57.30 m），
#: 且 JSON 的 tf 欄位是單位變換（不是兩者之間的換算）。
#: 交接單寫「c27/c28/c3 的座標真值在 3F_info.csv」會誤導 —— 以 RViz 實際畫面為準。
ROUTING_STATION_JSON = "src/campusrover_routing/share/json/itc_3f_3.json"

#: routing_engine_node 的幾何節點表（`file_node_info` 參數）。
#: 欄位序由 module_linkage.cpp:114-120 確認為 ``name, x, y, z, qw, qx, qy, qz``
#: —— 注意 **qw 在前**。保留供對照，但**生成點與導航目標請用 JSON**。
ROUTING_INFO_CSV = "src/campusrover_routing/share/node_module/3F_info.csv"

#: 模擬起始的 routing 站（用 ROUTING_STATION_JSON 的站名）。
SPAWN_ROUTING_NODE: str = "c28"

#: 生成時的朝向：面向這個站。JSON 的 rooms 沒有朝向資訊（rw=1, rz=0），
#: 所以用「面向走廊下一站」來決定 yaw，否則車會朝著牆生出來。
SPAWN_FACING_NODE: str = "c4"


def _rot(yaw: float):
    """row-vector 旋轉矩陣：v @ _rot(yaw) 等於把 v 旋轉 yaw。"""
    import math
    c, s = math.cos(yaw), math.sin(yaw)
    return ((c, s), (-s, c))


def world_to_map(x: float, y: float, yaw: float = 0.0) -> tuple[float, float, float]:
    """USD World 座標 → NDT map 座標。"""
    R = _rot(WORLD_TO_MAP_YAW_RAD)
    tx, ty = WORLD_TO_MAP_TRANSLATION
    return (x * R[0][0] + y * R[1][0] + tx,
            x * R[0][1] + y * R[1][1] + ty,
            yaw + WORLD_TO_MAP_YAW_RAD)


def map_to_world(x: float, y: float, yaw: float = 0.0) -> tuple[float, float, float]:
    """NDT map 座標 → USD World 座標。"""
    R = _rot(-WORLD_TO_MAP_YAW_RAD)
    tx, ty = WORLD_TO_MAP_TRANSLATION
    dx, dy = x - tx, y - ty
    return (dx * R[0][0] + dy * R[1][0],
            dx * R[0][1] + dy * R[1][1],
            yaw - WORLD_TO_MAP_YAW_RAD)


def read_station_nodes(json_path) -> dict[str, tuple[float, float, float]]:
    """讀 itc_3f_3.json 的 rooms，回傳 {站名: (x, y, yaw)}（map frame）。

    這是導航堆疊實際使用的表 —— 見 ROUTING_STATION_JSON 的說明。
    """
    import json as _json
    import math
    from pathlib import Path

    d = _json.loads(Path(json_path).read_text())
    out: dict[str, tuple[float, float, float]] = {}
    for r in d.get("rooms", []):
        pos = r["position"]
        out[r["room"]] = (float(pos["x"]), float(pos["y"]),
                          2.0 * math.atan2(float(pos.get("rz", 0.0)), float(pos.get("rw", 1.0))))
    return out


def read_routing_nodes(csv_path) -> dict[str, tuple[float, float, float]]:
    """讀 3F_info.csv，回傳 {節點名: (x, y, yaw)}（map frame）。

    ⚠ 這是 routing_engine 的幾何表，**與導航站名表不同座標系**。
    生成點與導航目標請改用 read_station_nodes()。
    """
    import math
    from pathlib import Path

    nodes: dict[str, tuple[float, float, float]] = {}
    for line in Path(csv_path).read_text().splitlines():
        parts = line.strip().split(",")
        if len(parts) < 8:
            continue
        try:
            x, y = float(parts[1]), float(parts[2])
            qw, qz = float(parts[4]), float(parts[7])
        except ValueError:
            continue
        nodes[parts[0]] = (x, y, 2.0 * math.atan2(qz, qw))
    return nodes


# --------------------------------------------------------------------------
# 8. 走廊障礙物
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class Obstacle:
    """走廊裡的一個障礙物，位置用 **map frame** 指定（與 routing 站同一個座標系）。

    ⚠ 為什麼用帶碰撞體的幾何代理而不是 People 角色：
    論文 §2.8.2 已載明「型別為 Lidar 的 RTX 光達不對骨架網格角色做光線追蹤」。
    本專案用的是 **PhysX 光達**，它對**物理碰撞體**做 raycast —— 骨架動畫角色
    沒有碰撞體，**同樣照不到**。而且 People 資產是抓不到的 S3 連結。
    放上去只會得到「看得見但偵測不到、車直接穿過」的行人，對導航測試是負面的。
    """

    name: str
    map_x: float
    map_y: float
    #: "person" = 直立圓柱（人體代理）；"box" = 方箱（推車／雜物）
    kind: str = "person"
    radius: float = 0.25          # 人體代理半徑（肩寬約 0.45~0.5 m）
    height: float = 1.70          # 成人身高
    size_x: float = 0.6           # box 用
    size_y: float = 0.6
    yaw_deg: float = 0.0


#: 預設障礙物：沿 c28 → c25 這條展示路線佈置，讓車真的要閃避。
#: c28 在 map (-0.058, +5.950)、c25 在 (-14.218, +5.377)，走廊沿 map -x 方向。
DEFAULT_OBSTACLES: tuple[Obstacle, ...] = (
    Obstacle("person_a", -3.0, 6.3, "person"),
    Obstacle("person_b", -6.0, 5.4, "person"),
    Obstacle("person_c", -9.5, 6.2, "person"),
    # ⚠ 高度必須 > sensor_h - z_filter = 1.43 - 0.5 = 0.93 m，否則整個落在
    #   policy 可見帶下方 —— 有碰撞體（車撞得到）但 72 維觀測裡不存在。
    #   原本 0.90 / 1.00 m 就是這種「撞得到但看不到」的組合，已抬高。
    #   驗收見 obstacle_motion.lidar_visible_height 與其測試。
    Obstacle("cart_a", -4.6, 5.3, "box", size_x=0.7, size_y=0.5, height=1.15),
    Obstacle("cart_b", -11.5, 5.8, "box", size_x=0.8, size_y=0.6, height=1.25),
)


@dataclass(frozen=True)
class MovingObstacle:
    """沿折線等速往返的動態障礙物（行人／推車）。

    與 :class:`Obstacle` 同樣用帶碰撞體的幾何代理，理由見該類別的說明：
    PhysX 光達對**碰撞體** raycast，骨架動畫角色沒有碰撞體照不到。
    差別只在這個會動 —— USD 端額外套 RigidBodyAPI 並設為 kinematic，
    位置由 run_isaac_sim 每個物理步依 obstacle_motion.position_at 更新。

    ⚠ 為什麼不追求人體外形：policy 吃的是 72-bin sweep（每 bin 5°）且
    前處理只保留地板上方 [0.93, 1.93] m 的水平帶。5 m 處的軀幹只佔約
    1 個 bin，取 min-pool 後圓柱與人體網格的輸出完全相同。
    """

    name: str
    #: map frame 的 (x, y) 路徑點，至少兩個才會動。
    waypoints: tuple[tuple[float, float], ...] = ()
    #: 疊在碰撞圓柱上的人物網格資產名（PEOPLE_ASSETS 的 key）。
    #: None = 不套外殼，直接顯示圓柱。
    visual_asset: str | None = "F_Business_02"
    #: 行進速率 m/s。成人平均步行 1.2~1.4；推車或長者取 0.6~0.9。
    speed: float = 1.2
    #: 終點行為，見 obstacle_motion.MODES。
    mode: str = "pingpong"
    #: 起始時間偏移 s —— 讓多個行人不同相位，避免整齊劃一。
    phase_s: float = 0.0
    kind: str = "person"
    radius: float = 0.25
    height: float = 1.70
    size_x: float = 0.6
    size_y: float = 0.6


#: NVIDIA People 角色資產（Isaac Sim 5.1 CDN）。這些是**純視覺**外殼：
#: 骨架網格沒有碰撞體，PhysX 光達照不到，所以底下一定要墊碰撞代理。
PEOPLE_ASSET_BASE = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/5.1/Isaac/People/Characters"
)
PEOPLE_ASSETS: dict[str, str] = {
    "F_Business_02": f"{PEOPLE_ASSET_BASE}/F_Business_02/F_Business_02.usd",
    "F_Medical_01": f"{PEOPLE_ASSET_BASE}/F_Medical_01/F_Medical_01.usd",
    "M_Medical_01": f"{PEOPLE_ASSET_BASE}/M_Medical_01/M_Medical_01.usd",
}



#: 預設行人。走廊中心線約 (0,+6) → (-10,+5) → (-17,+3.6)，寬約 2.9 m。
#: 三種互動型態各一，對應論文的 crossing / head_on / same_direction 分類。
DEFAULT_MOVING_OBSTACLES: tuple[MovingObstacle, ...] = (
    # 橫穿：垂直切過走廊，車必須讓或繞。相位 0 → 車出發後最早遇到。
    MovingObstacle(
        "walker_cross",
        waypoints=((-5.0, 4.3), (-5.0, 6.6)),
        speed=1.2, mode="pingpong", phase_s=0.0, visual_asset="F_Business_02",
    ),
    # 迎面：沿走廊往東走向出發點，與車對向。錯開相位避免同步。
    MovingObstacle(
        "walker_headon",
        waypoints=((-13.0, 5.6), (-2.5, 5.9)),
        speed=1.1, mode="pingpong", phase_s=3.0, visual_asset="M_Medical_01",
    ),
    # 同向較慢：車從後方接近，需要超車或跟隨。
    MovingObstacle(
        "walker_slow",
        waypoints=((-6.5, 4.8), (-14.5, 4.4)),
        speed=0.6, mode="pingpong", phase_s=1.5, visual_asset="F_Medical_01",
    ),
)


# --------------------------------------------------------------------------
# 9. RL checkpoint profile
# --------------------------------------------------------------------------

#: 每個 checkpoint 與其**配套設定**綁成一組。
#:
#: ⚠ 為什麼一定要綁：這些模型的觀測契約不同。
#:   sa6_tc_dense_420k : raw_obs 139、無 action stacking
#:   sa1r1 / sa4r2 / sa4r3 / sa5r2 : raw_obs **83** + action stacking frame_stack=8
#:                                   （lidar_hist 504、rl_input 179）
#:   只換 model_path 而沿用舊 yaml，policy 會拿到錯誤維度的觀測 ——
#:   **不會報錯，只會安靜地算出垃圾動作**。
#:
#: 車端評語摘要（來自 deploy_select.sh 的選單）：
#:   sa1r1  ★ c1700 · parity 過 · 僅空曠（HIDE_TS 內但有 config）
#:   sa4r2  ⚠ 僅正面對衝走廊 98.47%/CR 1.53%；走廊橫穿 83.05%/16.95% ❌；空曠未量測
#:          🔴 SA4 on HOLD，用途很窄
#:   sa4r3  ⚠ 診斷跑：空曠 94.06%/5.94% ✅；橫穿 81.13%/18.87% ❌；對衝 78.21%/21.73% ❌
#:          🔴 無任何 SA4 通過聯合閘門（accepted_parent=null）
#:   sa5r2  ⚠ 未過閘門：窄縫 100%/0%；空曠 91.46%；走廊 random_2d CR 67.44% ❌
#:          ⚠ 此血緣 speed_rate 0.7 差於 1.0（CR 38.82%→58.77%）
RL_PROFILES: dict[str, tuple[str, str, str]] = {
    # profile: (checkpoint, policy yaml, preprocessor yaml)
    "sa6":   ("sa6_tc_dense_420k.ts",
              "policy_params.yaml", "lidar_preprocessor_params.yaml"),
    "sa1r1": ("sa1r1_c1700_83d_k8.ts",
              "policy_params_sa1r1.yaml", "lidar_preprocessor_params_sa1r1.yaml"),
    "sa4r2": ("sa4_r2_c6400_83d_k8.ts",
              "policy_params_sa4r2.yaml", "lidar_preprocessor_params_sa4r2.yaml"),
    "sa4r3": ("sa4_r3_it50_83d_k8.ts",
              "policy_params_sa4r3.yaml", "lidar_preprocessor_params_sa4r3.yaml"),
    "sa5r2": ("sa5r2_c250_k8_e2e_32000.ts",
              "policy_params_sa5r2c250.yaml", "lidar_preprocessor_params_sa5r2c250.yaml"),
}

#: 預設 profile。使用者 2026-09-21 指定改用車端選單上的 SA4/SA5 系列。
DEFAULT_RL_PROFILE: str = "sa4r2"
