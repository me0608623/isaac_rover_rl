"""build_ros_graph 的整合測試。

重點是**驗結果、不驗算式**：例如光達位置不檢查「有沒有呼叫矩陣乘法」，
而是把 override layer 疊起來後實際量 base_link→velodyne 的距離。
row-vector / column-vector 乘錯方向這類錯誤只有這樣才擋得住。

執行：
    python3 -m pytest sim_ws/scripts/test_build_ros_graph.py -v
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
from pxr import Usd, UsdGeom

import build_ros_graph as B
import ros_graph_spec as S

SOURCE_USD = Path("/home/aa/Ros/charge_rl/assets/3F/3floor_ver_1.usd")

pytestmark = pytest.mark.skipif(not SOURCE_USD.exists(), reason="來源 USD 不在此機器上")


@pytest.fixture(scope="module")
def built(tmp_path_factory) -> Path:
    out = tmp_path_factory.mktemp("usd") / "fixed.usda"
    spec = S.SimRosSpec().with_tf_ownership(S.default_tf_ownership())
    B.build(SOURCE_USD, out, spec)
    return out


@pytest.fixture(scope="module")
def stage(built: Path) -> Usd.Stage:
    return Usd.Stage.Open(str(built))


# --------------------------------------------------------------------------
# 原始檔必須零改動
# --------------------------------------------------------------------------

def test_source_usd_is_never_modified(built: Path):
    """override layer 的全部價值就建立在這一條上。"""
    before = hashlib.md5(SOURCE_USD.read_bytes()).hexdigest()
    spec = S.SimRosSpec().with_tf_ownership(S.default_tf_ownership())
    B.build(SOURCE_USD, built.parent / "again.usda", spec)
    assert hashlib.md5(SOURCE_USD.read_bytes()).hexdigest() == before


def test_output_is_ascii_and_reviewable(built: Path):
    """輸出必須是 ASCII usda，否則失去 git diff 的意義。"""
    head = built.read_bytes()[:8]
    assert head.startswith(b"#usda"), f"輸出不是 ASCII USD: {head!r}"


# --------------------------------------------------------------------------
# 幾何：驗量測結果，不驗算式
# --------------------------------------------------------------------------

def test_base_footprint_to_velodyne_matches_urdf(stage: Usd.Stage):
    """實車 xacro:117 (0,0,0.13) + xacro:135 (-0.02,0,1.3) = (-0.02, 0, 1.43)。

    錨定 base_footprint 而非 base_link，因為 IsaacComputeOdometry 的 chassisPrim
    就是 base_footprint，TF 樹的 odom→base_footprint 對應的實體是那顆 prim。

    原始 USD 為 (-0.0805, 0, 1.4724)。這條測試擋下兩種曾實際發生的錯誤：
      1. row-vector 乘錯方向 → x 算成 -0.03196，與正解差 3.6 cm
      2. 錨定 base_link → 殘留 3.66 mm（USD 的 base_footprint→base_link 是 0.1336）
    """
    measured = B.measure_base_footprint_to_velodyne(stage)
    expected = B.spec_offset(S.SimRosSpec())
    assert measured[0] == pytest.approx(expected[0], abs=1e-6)
    assert measured[1] == pytest.approx(expected[1], abs=1e-6)
    assert measured[2] == pytest.approx(expected[2], abs=1e-6)


def test_velodyne_height_above_base_footprint_is_1_43m(stage: Usd.Stage):
    """端到端檢查：感測器離地高度必須是實車的 1.43 m。

    這個數字直接決定模擬點雲能不能對上 3F_314.pcd（該圖是感測器在 1.43 m 錄的）。
    """
    cache = UsdGeom.XformCache()
    fp = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(f"{B.ROBOT}/base_footprint"))
    velo = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(B.P_VELODYNE))
    height = (velo * fp.GetInverse()).ExtractTranslation()[2]
    assert height == pytest.approx(1.43, abs=1e-6)


def test_velodyne_orientation_is_untouched(stage: Usd.Stage):
    """velodyne 與其下 Lidar 各帶 180° Z 旋轉、兩者相消。

    動了旋轉會讓點雲繞 Z 轉 180°，NDT 直接失效。
    """
    src = Usd.Stage.Open(str(SOURCE_USD))
    a = src.GetPrimAtPath(B.P_VELODYNE).GetAttribute("xformOp:orient").Get()
    b = stage.GetPrimAtPath(B.P_VELODYNE).GetAttribute("xformOp:orient").Get()
    assert a == b


# --------------------------------------------------------------------------
# ROS 介面
# --------------------------------------------------------------------------

def test_point_cloud_frame_is_velodyne_link(stage: Usd.Stage):
    node = stage.GetPrimAtPath(f"{B.G_VELO}/ros2_publish_point_cloud")
    assert node.GetAttribute("inputs:frameId").Get() == "velodyne_link"


def test_all_sensor_frames_are_distinct(stage: Usd.Stage):
    """原始檔把 4 個感測器全標成 base_link。"""
    nodes = [
        f"{B.G_VELO}/ros2_publish_point_cloud",
        f"{B.G_IMU}/ros2_publish_imu",
        f"{B.G_2D}/publish_front_2d_lidar_scan",
        f"{B.G_2D}/publish_back_2d_lidar_scan",
    ]
    frames = [stage.GetPrimAtPath(n).GetAttribute("inputs:frameId").Get() for n in nodes]
    assert len(set(frames)) == len(frames), f"frame 重複: {frames}"
    assert "base_link" not in frames


def test_sim_publishes_odom_gt_not_odom(stage: Usd.Stage):
    """留 /odom 給 odom_drift_injector 輸出。"""
    node = stage.GetPrimAtPath(f"{B.G_ODOM}/ros2_publish_odometry")
    assert node.GetAttribute("inputs:topicName").Get() == "/odom_gt"


def test_isaac_publishes_no_tf(stage: Usd.Stage):
    """方案 A：TF 全交給 injector 與 robot_state_publisher。

    ⚠ 這條測試原本只驗 ``prim.IsActive()``，**通過了卻擋不住真實問題**：
    2026-09-21 實際跑 Isaac Sim 時，四個節點照樣發出 7 組 TF。
    OmniGraph 不看 prim 的 active 狀態。改驗真正決定行為的兩件事：
    execIn 是否斷開，以及 topicName 是否已導向死路。
    """
    for node_path in B.TF_PUBLISHER_NODES:
        prim = stage.GetPrimAtPath(node_path)
        exec_in = prim.GetAttribute("inputs:execIn")
        conns = [str(c) for c in exec_in.GetConnections()] if exec_in and exec_in.IsValid() else []
        assert not conns, f"{node_path} 的 execIn 仍連著 {conns} —— 會被求值並發 TF"
        assert prim.GetAttribute("inputs:topicName").Get() == B.DEAD_TF_TOPIC


def test_two_d_lidars_are_disabled_by_default(stage: Usd.Stage):
    """/scan 與 /back_scan 實測沒有資料（RPLidar_S2E 資產抓不到），

    但節點每幀仍在建 render product 耗 GPU。預設關掉。
    """
    assert S.SimRosSpec().enable_2d_lidars is False
    for node_path in B.TWO_D_LIDAR_NODES:
        prim = stage.GetPrimAtPath(node_path)
        exec_in = prim.GetAttribute("inputs:execIn")
        if exec_in and exec_in.IsValid():
            assert not exec_in.GetConnections(), f"{node_path} 仍會被求值"


def test_navmesh_floor_keeps_collision(stage: Usd.Stage):
    """NavFloor 的碰撞必須保留 —— 它是走廊大部分區域唯一的可行走面。

    ⚠ 2026-09-21 端到端實測的回歸：先前為了不讓光達打到假地板而把碰撞關掉，
    結果車從 c28 開約 10 m 到 World(-17.3,+7.8) 就掉出世界（z 掉到 -7102 m），
    因為走廊 mesh Mesh_015 在那裡沒有地板幾何。
    """
    for path in B.NAV_FLOOR_PRIMS:
        prim = stage.GetPrimAtPath(path)
        assert prim.IsValid(), path
        enabled = prim.GetAttribute("physics:collisionEnabled")
        assert enabled.Get() is not False, f"{path} 碰撞被關掉 → 車會掉出世界"


def test_navmesh_floor_top_aligns_with_corridor_floor(stage: Usd.Stage):
    """NavFloor 上表面必須與走廊地板重合，否則光達會看到第二層假地板。"""
    from pxr import UsdGeom as _UG
    floor_top = B.measure_corridor_floor_top(stage)
    cache = _UG.XformCache()
    for path in B.NAV_FLOOR_PRIMS:
        prim = stage.GetPrimAtPath(path)
        half = float(prim.GetAttribute("size").Get() or 2.0) / 2.0
        scale_z = float(prim.GetAttribute("xformOp:scale").Get()[2])
        tz = float(prim.GetAttribute("xformOp:translate").Get()[2])
        assert tz + half * scale_z == pytest.approx(floor_top, abs=1e-6), path


def test_corridor_floor_measurement_uses_mode_not_median(stage: Usd.Stage):
    """地板高度必須量到 -0.323（執行期 base_footprint 實測 -0.3229）。

    該 mesh 在地板帶內有三層（-0.61 板底 / -0.32 地板 / -0.17 門檻），
    用中位數或「上半部中位數」會算成 -0.173，差 15 cm。
    """
    assert B.measure_corridor_floor_top(stage) == pytest.approx(-0.3233, abs=2e-3)


def test_isaac_publishes_ideal_cloud_for_smear_injection(stage: Usd.Stage):
    """啟用拖影注入時，Isaac 要讓出 /velodyne_points 給 lidar_motion_smear。

    與 odom 同樣的分層。若 Isaac 直接佔用 /velodyne_points，
    拖影節點無處插入，NDT 的主要誤差源就不存在於模擬中。
    """
    node = stage.GetPrimAtPath(f"{B.G_VELO}/ros2_publish_point_cloud")
    topic = node.GetAttribute("inputs:topicName").Get()
    spec = S.SimRosSpec()
    assert spec.inject_lidar_motion_smear is True
    assert topic == spec.topics.point_cloud_ideal
    assert topic != spec.topics.point_cloud


def test_degrader_chain_has_no_topic_collisions():
    """三層降級鏈的 topic 不可互撞，否則會形成回授或搶發。

        Isaac /odom_gt            -> odom_drift_injector   -> /odom
        Isaac /velodyne_points_ideal -> lidar_motion_smear -> /velodyne_points
    """
    t = S.TopicNames()
    produced_by_isaac = {t.odom_ground_truth, t.point_cloud_ideal, t.clock, t.imu,
                         t.scan_front, t.scan_back}
    produced_by_degraders = {"/odom", t.point_cloud}
    assert not (produced_by_isaac & produced_by_degraders)


def test_velodyne_physics_joint_matches_urdf(stage: Usd.Stage):
    """執行期生效的是**物理關節**，不是 xformOp。

    ⚠ 2026-09-21 實測：velodyne 帶 PhysicsRigidBodyAPI，PhysX 會依
    velodyne_base_mount_joint 的 localPos 重新擺位，蓋掉 xformOp:translate。
    只驗 Xform 的測試會給出假陽性。
    """
    import math
    fp = stage.GetPrimAtPath(B.P_FOOTPRINT_JOINT).GetAttribute("physics:localPos0").Get()
    vj = stage.GetPrimAtPath(B.P_VELO_JOINT).GetAttribute("physics:localPos0").Get()
    # base_footprint → velodyne = (base_link→velodyne) − (base_link→base_footprint)
    got = (vj[0] - fp[0], vj[1] - fp[1], vj[2] - fp[2])
    want = B.spec_offset(S.SimRosSpec())
    for g, w, axis in zip(got, want, "xyz"):
        assert g == pytest.approx(w, abs=1e-5), f"{axis}: {g} != {w}"


def test_xform_and_physics_joint_agree(stage: Usd.Stage):
    """Xform 與物理關節必須給出一致的感測器位置，否則播放瞬間會跳一下。"""
    xform_offset = B.measure_base_footprint_to_velodyne(stage)
    fp = stage.GetPrimAtPath(B.P_FOOTPRINT_JOINT).GetAttribute("physics:localPos0").Get()
    vj = stage.GetPrimAtPath(B.P_VELO_JOINT).GetAttribute("physics:localPos0").Get()
    joint_z = vj[2] - fp[2]
    assert xform_offset[2] == pytest.approx(joint_z, abs=2e-3), \
        f"Xform 給 {xform_offset[2]:.4f}，關節給 {joint_z:.4f}"


# --------------------------------------------------------------------------
# 以下為 2026-09-21 編輯時被誤刪、已復原的測試
# --------------------------------------------------------------------------

def test_clock_publisher_exists_and_uses_simulation_time(stage: Usd.Stage):
    pub = stage.GetPrimAtPath(f"{B.CLOCK_GRAPH}/ros2_publish_clock")
    assert pub.IsValid()
    assert pub.GetAttribute("node:type").Get() == "isaacsim.ros2.bridge.ROS2PublishClock"
    conns = [str(c) for c in pub.GetAttribute("inputs:timeStamp").GetConnections()]
    assert any("isaac_read_simulation_time" in c for c in conns), conns


def test_no_publisher_stamps_from_playback_time(stage: Usd.Stage):
    """播放時間與 /clock 不同源，會讓 NDT 的 lookupTransform 全數丟 extrapolation。"""
    offenders = []
    for prim in stage.Traverse():
        attr = prim.GetAttribute("inputs:timeStamp")
        if not attr or not attr.IsValid():
            continue
        exec_in = prim.GetAttribute("inputs:execIn")
        if exec_in and exec_in.IsValid() and not exec_in.GetConnections():
            continue  # execIn 斷開的節點永遠不會被求值
        for c in attr.GetConnections():
            if "on_playback_tick" in str(c) and str(c).endswith("outputs:time"):
                offenders.append(f"{prim.GetPath()} <- {c}")
    assert not offenders, "仍有 publisher 使用 playback time:\n" + "\n".join(offenders)


def test_differential_controller_matches_vehicle(stage: Usd.Stage):
    node = stage.GetPrimAtPath(f"{B.G_DRIVE}/differential_controller")
    assert node.GetAttribute("inputs:wheelDistance").Get() == pytest.approx(0.559212)
    assert node.GetAttribute("inputs:maxAngularSpeed").Get() == pytest.approx(1.2)


def test_lidar_range_covers_ndt_crop(stage: Usd.Stage):
    lidar = stage.GetPrimAtPath(B.P_LIDAR)
    assert lidar.GetAttribute("maxRange").Get() >= S.NDT_CROP_RANGE_M


def test_rebuild_is_idempotent(tmp_path: Path):
    """重跑一次不應產生額外修改 —— 否則代表有操作沒有檢查既有狀態。"""
    spec = S.SimRosSpec().with_tf_ownership(S.default_tf_ownership())
    first = B.build(SOURCE_USD, tmp_path / "a.usda", spec)
    second = B.build(SOURCE_USD, tmp_path / "b.usda", spec)
    assert sum(len(c) for _, c in first) == sum(len(c) for _, c in second)
    assert (tmp_path / "a.usda").read_text() == (tmp_path / "b.usda").read_text()


CSV = Path("/home/aa/IsaacLab/sim_ws") / S.ROUTING_INFO_CSV
STATION_JSON = Path("/home/aa/IsaacLab/sim_ws") / S.ROUTING_STATION_JSON


def test_routing_csv_quaternion_column_order_is_qw_first():
    """module_linkage.cpp:117-120 讀的是 ow,ox,oy,oz —— **qw 在第 5 欄**。

    照 (qx,qy,qz,qw) 讀，c28 會被解讀成繞 X 翻 180°，車生出來是倒的。
    """
    import math
    assert math.degrees(S.read_routing_nodes(CSV)["c28"][2]) == pytest.approx(-89.29, abs=0.01)


def test_map_world_round_trip():
    import math
    for mx, my, myaw in [(-5.67, -0.49, -1.558), (0.0, 0.0, 0.0), (20.0, -33.0, 2.5)]:
        wx, wy, wyaw = S.map_to_world(mx, my, myaw)
        bx, by, byaw = S.world_to_map(wx, wy, wyaw)
        assert (bx, by) == pytest.approx((mx, my), abs=1e-9)
        assert math.sin(byaw - myaw) == pytest.approx(0.0, abs=1e-12)


def test_robot_spawns_at_c28(stage: Usd.Stage):
    """使用者指定：車要生在 routing c28 上。

    ⚠ 2026-09-21 修正：必須用 **itc_3f_3.json 的站表**，不是 3F_info.csv。
    兩套表同名但座標差 8.54 m。RViz 的 routes_visualization 畫的綠色圖
    （c1,c2,c4,c5,c9,c10,c24,c25,c26,c29,c33,c37…）正是 JSON 的 29 個 room，
    使用者從畫面上一眼看出車不在任何 routing 節點上。
    """
    import math
    x, y, z, yaw = B.measure_base_footprint_world_pose(stage)
    mx, my, _ = S.world_to_map(x, y, yaw)
    tx, ty, _ = S.read_station_nodes(STATION_JSON)["c28"]
    assert math.hypot(mx - tx, my - ty) < 1e-3, f"差 {math.hypot(mx-tx,my-ty):.3f} m"


def test_station_table_differs_from_csv_table():
    """兩套拓撲表同名不同座標，混用會讓車生在錯誤位置。

    這條測試把這個陷阱釘住，避免以後又改回 CSV。
    """
    import math
    st = S.read_station_nodes(STATION_JSON)
    csv = S.read_routing_nodes(CSV)
    d = math.hypot(st["c28"][0]-csv["c28"][0], st["c28"][1]-csv["c28"][1])
    assert d > 5.0, f"兩表 c28 只差 {d:.2f} m —— 若已統一，這條測試可移除"


def test_spawn_faces_along_corridor(stage: Usd.Stage):
    """JSON 的 rooms 沒有朝向（rw=1, rz=0），生成時須面向走廊下一站，

    否則車會朝著牆生出來。
    """
    import math
    x, y, _, yaw = B.measure_base_footprint_world_pose(stage)
    mx, my, myaw = S.world_to_map(x, y, yaw)
    st = S.read_station_nodes(STATION_JSON)
    fx, fy, _ = st[S.SPAWN_FACING_NODE]
    want = math.atan2(fy - my, fx - mx)
    assert abs(math.sin(myaw - want)) < 1e-3, \
        f"朝向 {math.degrees(myaw):.1f}° 不是面向 {S.SPAWN_FACING_NODE}（應為 {math.degrees(want):.1f}°）"


def test_spawn_sits_on_the_corridor_floor(stage: Usd.Stage):
    """base_footprint 是貼地接觸點，必須落在走廊地板上表面。

    執行期實測 base_footprint world z = -0.3229、走廊地板 -0.3233，差 0.4 mm。
    """
    _, _, z, _ = B.measure_base_footprint_world_pose(stage)
    assert z == pytest.approx(B.measure_corridor_floor_top(stage), abs=5e-3)


# --------------------------------------------------------------------------
# 走廊障礙物
# --------------------------------------------------------------------------

@pytest.fixture(scope="module")
def stage_with_obstacles(tmp_path_factory) -> Usd.Stage:
    from dataclasses import replace as _replace
    out = tmp_path_factory.mktemp("usd_obs") / "obs.usda"
    spec = S.SimRosSpec().with_tf_ownership(S.default_tf_ownership())
    spec = _replace(spec, obstacles=S.DEFAULT_OBSTACLES)
    B.build(SOURCE_USD, out, spec)
    return Usd.Stage.Open(str(out))


def test_obstacles_have_collision(stage_with_obstacles: Usd.Stage):
    """沒有碰撞體的障礙物，PhysX 光達打不到、車也撞不到 —— 等於不存在。

    ⚠ 這正是為什麼不用 omni.anim.people 的角色：論文 §2.8.2 已載明
    RTX Lidar 不對骨架網格做光線追蹤，而 PhysX 光達是對碰撞體 raycast，
    骨架角色沒有碰撞體 → 同樣照不到。
    """
    from pxr import UsdPhysics
    st = stage_with_obstacles
    n = 0
    for o in S.DEFAULT_OBSTACLES:
        prim = st.GetPrimAtPath(f"{B.OBSTACLE_ROOT}/{o.name}")
        assert prim.IsValid(), o.name
        assert prim.HasAPI(UsdPhysics.CollisionAPI), f"{o.name} 沒有碰撞體"
        n += 1
    assert n == len(S.DEFAULT_OBSTACLES)


def test_obstacles_sit_on_the_floor(stage_with_obstacles: Usd.Stage):
    """障礙物底部必須貼在走廊地板上，不能浮空或埋進地板。"""
    from pxr import UsdGeom as _UG
    st = stage_with_obstacles
    floor = B.measure_corridor_floor_top(st)
    cache = _UG.XformCache()
    for o in S.DEFAULT_OBSTACLES:
        prim = st.GetPrimAtPath(f"{B.OBSTACLE_ROOT}/{o.name}")
        z = cache.GetLocalToWorldTransform(prim).ExtractTranslation()[2]
        bottom = z - o.height / 2.0
        assert bottom == pytest.approx(floor, abs=1e-3), f"{o.name} 底部 {bottom:.3f} vs 地板 {floor:.3f}"


def test_obstacles_are_in_navigable_space():
    """障礙物要在走廊裡才有意義 —— 放在牆裡等於沒放。"""
    import numpy as np
    sampled = Path("/tmp/claude-1001/-home-aa-IsaacLab/c91a54e7-f2a8-4332-9178-234b00ffefba/scratchpad/usd_sampled.npy")
    if not sampled.exists():
        pytest.skip("取樣點雲不在")
    usd = np.load(sampled)
    floor = -0.3233
    band = usd[(usd[:, 2] > floor + 0.10) & (usd[:, 2] < floor + 1.60)]
    for o in S.DEFAULT_OBSTACLES:
        wx, wy, _ = S.map_to_world(o.map_x, o.map_y, 0.0)
        d = float(np.hypot(band[:, 0] - wx, band[:, 1] - wy).min())
        r = o.radius if o.kind == "person" else max(o.size_x, o.size_y) / 2
        assert d > r + 0.3, f"{o.name} 離牆只有 {d:.2f} m（半徑 {r:.2f}）"


def test_clean_corridor_when_obstacles_empty(stage: Usd.Stage):
    """預設 spec 不放障礙物 —— 做定位基準時走廊要是淨空的。"""
    assert S.SimRosSpec().obstacles == ()
    assert not stage.GetPrimAtPath(B.OBSTACLE_ROOT).IsValid()

