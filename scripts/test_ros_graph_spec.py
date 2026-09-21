"""ros_graph_spec 的回歸守門測試。

這些測試的用途不是「驗證程式碼能跑」，而是把車端契約釘死：
任何人若把模擬的 frame 名稱、輪距、光達參數改成與實車不一致，這裡會擋下來。

執行：
    python3 -m pytest sim_ws/scripts/test_ros_graph_spec.py -v
"""

from __future__ import annotations

import math

import pytest

import ros_graph_spec as spec


# --------------------------------------------------------------------------
# Frame 契約 —— 對照車端 yaml / xacro
# --------------------------------------------------------------------------

def test_velodyne_frame_matches_vehicle_driver():
    """velodyne_points.yaml:10 frame_id: velodyne_link。

    這是原 USD 最致命的一條：點雲被標成 base_link，會讓 NDT 把雲放到
    離地 0.13 m，而 3F_314.pcd 是感測器在 1.43 m 錄的。
    """
    assert spec.FRAMES.velodyne == "velodyne_link"
    assert spec.FRAMES.velodyne != spec.FRAMES.base_link


def test_odom_tf_child_is_base_footprint_not_base_link():
    """driver_chgh.yaml: ros.base_frame: base_footprint。

    原 USD 發的是 odom→base_link，與實車方向相反。
    """
    assert spec.FRAMES.base_footprint == "base_footprint"
    edges = spec.expected_static_tf_edges(spec.SimRosSpec())
    assert edges[spec.FRAMES.base_link] == spec.FRAMES.base_footprint


def test_two_dimensional_lidars_have_distinct_frames():
    """原 USD 把前後兩顆 RPLIDAR 都標成 base_link。"""
    assert spec.FRAMES.ydlidar_front != spec.FRAMES.ydlidar_back
    assert spec.FRAMES.ydlidar_front == "ydlidar_front_link"
    assert spec.FRAMES.ydlidar_back == "ydlidar_back_link"


def test_every_frame_has_exactly_one_parent():
    """TF 樹不得有雙 parent，否則 tf2 會交替接受兩份資料且不報錯。"""
    edges = spec.expected_static_tf_edges(spec.SimRosSpec())
    assert len(edges) == len(set(edges.keys()))
    assert spec.FRAMES.base_footprint not in edges, "base_footprint 的 parent 應由 odom 側提供"


# --------------------------------------------------------------------------
# Topic 契約
# --------------------------------------------------------------------------

def test_sim_does_not_occupy_odom_topic():
    """odom_drift_injector 吃 /odom_gt 吐 /odom（交接單 §5.4）。

    模擬若直接發 /odom，injector 無處插入，漂移注入整個失效。
    """
    assert spec.TOPICS.odom_ground_truth == "/odom_gt"
    assert spec.TOPICS.odom_ground_truth != "/odom"


def test_clock_is_published():
    """交接單 §4：整條鏈 use_sim_time:=true，前提是 Isaac 要發 /clock。

    原 USD 的 5 張 graph 裡完全沒有 ROS2PublishClock。
    """
    s = spec.SimRosSpec()
    assert s.publish_clock is True
    assert spec.TOPICS.clock == "/clock"


def test_stamps_use_simulation_time_not_playback_time():
    """原 USD 把所有 timeStamp 接到 OnPlaybackTick.outputs:time，

    而每張 graph 裡的 IsaacReadSimulationTime 節點是懸空的。
    playback time 與 /clock 不同源 → NDT 的 lookupTransform 會全數丟 extrapolation。
    """
    assert spec.SimRosSpec().use_simulation_time_for_stamps is True


# --------------------------------------------------------------------------
# 幾何契約 —— 對照 campusrover_chgh.xacro
# --------------------------------------------------------------------------

def test_velodyne_height_above_base_footprint_is_1_43m():
    """xacro: base_footprint→base_link 0.13 + base_link→velodyne 1.3 = 1.43。

    USD 實測為 1.4724（高 42.4 mm），必須被 align_sensor_prims_to_urdf() 修正。
    """
    total_z = spec.BASE_FOOTPRINT_TO_BASE_LINK_M[2] + spec.BASE_LINK_TO_VELODYNE_M[2]
    assert total_z == pytest.approx(1.43, abs=1e-9)


def test_velodyne_x_offset_matches_urdf():
    """xacro:135 origin xyz="-0.02 0 1.3"。USD 實測 -0.0846。"""
    assert spec.BASE_LINK_TO_VELODYNE_M[0] == pytest.approx(-0.02, abs=1e-9)


# --------------------------------------------------------------------------
# 底盤契約 —— 對照 driver_chgh.yaml
# --------------------------------------------------------------------------

def test_wheel_distance_matches_vehicle_not_usd_default():
    """原 USD DifferentialController wheelDistance=0.7。

    既不等於實車 0.559212，也不等於 USD 自身輪關節間距 0.554
    （left/right_wheel_joint localPos0 y = ±0.27700004）。
    誤差會讓實際 ω = 指令 ω × 0.7/0.554 ≈ 1.26。
    """
    drive = spec.DifferentialDriveSpec()
    assert drive.wheel_distance_m == pytest.approx(0.559212, abs=1e-9)
    assert drive.wheel_distance_m != 0.7


def test_wheel_radius_is_mean_of_two_measured_diameters():
    """driver_chgh.yaml: left 0.244211 / right 0.239577（左右不對稱 1.92%）。

    差速控制器只吃單一半徑，取兩者均值；不對稱本身由
    odom_drift_injector 的 e_d 參數建模，不在這裡處理。
    """
    drive = spec.DifferentialDriveSpec()
    assert drive.wheel_radius_m == pytest.approx((0.244211 + 0.239577) / 4.0, abs=1e-12)
    assert drive.wheel_radius_m != 0.134


def test_angular_speed_capped_at_chassis_limit():
    """driver_chgh.yaml: profile_omega_max 1.2。原 USD 給 3.0。

    模擬若允許實車做不到的轉速，policy 會學到無法部署的行為。
    """
    assert spec.DifferentialDriveSpec().max_angular_speed_rad_s == pytest.approx(1.2)


# --------------------------------------------------------------------------
# 光達契約
# --------------------------------------------------------------------------

def test_lidar_geometry_is_vlp16():
    lidar = spec.LidarSpec()
    assert lidar.num_rings == 16
    assert lidar.points_per_ring == 1800
    assert lidar.rotation_rate_hz == pytest.approx(10.0)


def test_lidar_max_range_covers_ndt_crop():
    """NDT 的 voxel filter crop 是 ±40 m（交接單 §3）。

    原 USD maxRange=20 會讓走廊遠端結構在進 NDT 前就消失。
    """
    lidar = spec.LidarSpec()
    assert lidar.max_range_m >= spec.NDT_CROP_RANGE_M
    assert lidar.max_range_m > 20.0


# --------------------------------------------------------------------------
# TF 權責 —— 待使用者決策
# --------------------------------------------------------------------------

def test_tf_ownership_decision_is_made():
    """default_tf_ownership() 必須回傳一個決策，不能留 NotImplementedError。"""
    ownership = spec.default_tf_ownership()
    assert isinstance(ownership, spec.TfOwnership)


def test_odom_tf_has_single_publisher():
    """odom→base_footprint 只能有一個發佈者。

    odom_drift_injector 預設 publish_tf=true。若 Isaac 也發，
    tf2 會在真值與漂移值之間交替，NDT 解出來的 map→odom 會隨機跳。
    """
    ownership = spec.default_tf_ownership()
    injector_publishes_odom_tf = True  # odom_drift_injector 預設值
    assert not (ownership.isaac_publishes_odom_tf and injector_publishes_odom_tf), (
        "Isaac 與 odom_drift_injector 不可同時發 odom→base_footprint"
    )


# --------------------------------------------------------------------------
# RL checkpoint profile
# --------------------------------------------------------------------------

from pathlib import Path as _Path  # noqa: E402

ROVER_RL = _Path("/home/aa/IsaacLab/rover_rl")


@pytest.mark.parametrize("name", list(spec.RL_PROFILES))
def test_profile_files_exist(name):
    """checkpoint 與兩份配套 yaml 都必須存在，否則 launch 會在半路炸掉。"""
    ckpt, policy_yaml, preproc_yaml = spec.RL_PROFILES[name]
    assert (ROVER_RL / "models" / ckpt).exists(), ckpt
    cfg = ROVER_RL / "src" / "rover_rl_bringup" / "config"
    assert (cfg / policy_yaml).exists(), policy_yaml
    assert (cfg / preproc_yaml).exists(), preproc_yaml


@pytest.mark.parametrize("name", list(spec.RL_PROFILES))
def test_profile_yaml_points_at_its_own_checkpoint(name):
    """policy yaml 裡的 model_path 必須指向同一個 profile 的 checkpoint。

    ⚠ 這是防「換了模型但 yaml 沒跟上」的守門：這些模型的觀測契約不同
    （sa6 raw_obs 139 無 action stacking；其餘 raw_obs 83 + frame_stack 8），
    錯配**不會報錯，只會安靜地算出垃圾動作**。
    """
    ckpt, policy_yaml, _ = spec.RL_PROFILES[name]
    text = (ROVER_RL / "src" / "rover_rl_bringup" / "config" / policy_yaml).read_text()
    assert ckpt in text, f"{policy_yaml} 的 model_path 沒有指向 {ckpt}"


@pytest.mark.parametrize("name", [n for n in spec.RL_PROFILES if n != "sa6"])
def test_83d_profiles_have_matching_obs_spec(name):
    """83D 系列的 sidecar 必須一致宣告 raw_obs 83 + frame_stack 8。"""
    import json
    ckpt = spec.RL_PROFILES[name][0]
    sidecar = ROVER_RL / "models" / (ckpt.replace(".ts", ".obs_spec.json"))
    if not sidecar.exists():
        pytest.skip(f"{sidecar.name} 不在")
    d = json.loads(sidecar.read_text())
    assert d["raw_obs_dim"] == 83, d.get("raw_obs_dim")
    assert d.get("frame_stack") == 8, d.get("frame_stack")


def test_default_profile_is_known():
    assert spec.DEFAULT_RL_PROFILE in spec.RL_PROFILES
