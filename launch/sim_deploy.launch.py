"""模擬端部署堆疊 —— 與實車 deploy_full 對應的 sim 版本。

與實車的差異只有「誰提供感測與底盤」：
    實車： campusrover_driver + velodyne_driver          → /odom, /velodyne_points
    模擬： Isaac Sim + odom_drift_injector + motion_smear → /odom, /velodyne_points
其餘（NDT / routing / policy）**完全是同一套節點、同一份參數**。

TF 權責（方案 A，見 docs/2026-09-21_模擬ROS契約對照_PC端回覆.md §11）：
    map            → odom            ndt_localizer
    odom           → base_footprint  odom_drift_injector
    base_footprint → base_link → …   robot_state_publisher（實車 URDF）
    Isaac Sim 一條 TF 都不發。

用法：
    source sim_ws/setup_sim_env.sh
    ros2 launch sim_ws/launch/sim_deploy.launch.py
    ros2 launch sim_ws/launch/sim_deploy.launch.py enable_policy:=false   # 只跑定位
"""

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SIM_WS = Path(__file__).resolve().parents[1]
SCRIPTS = SIM_WS / "scripts"

# RL profile 定義與 World↔map 配準都在 scripts/ros_graph_spec.py（單一事實來源）
sys.path.insert(0, str(SCRIPTS))
import ros_graph_spec as S  # noqa: E402
ROVER_RL = Path("/home/aa/IsaacLab/rover_rl")
RL_CFG = ROVER_RL / "src" / "rover_rl_bringup" / "config"

#: PyTorch 裝在這裡而不是系統 Python —— 系統 python3.12（Jazzy）沒有 torch，
#: 而 conda env_isaaclab 的 torch 是 python3.11 用的，版本對不上。
#: 車端 policy_params.yaml 指定 device:"cpu"，所以裝與車端相同的 2.10.0+cpu。
VENDOR_PY = SIM_WS / "vendor" / "py312"


def _rl_env() -> dict:
    """rover_rl 節點的額外環境：把 vendor 的 torch 加進 PYTHONPATH。"""
    import os
    existing = os.environ.get("PYTHONPATH", "")
    return {"PYTHONPATH": f"{VENDOR_PY}:{existing}" if existing else str(VENDOR_PY)}

#: 與車端 deploy_full.launch.py 完全相同的 NDT 參數。
#: ⚠ max_iterations 程式碼預設 30，被車端 yaml 壓到 10；converged 門檻 1.6
#: 在 PCL 1.12→1.14 之間計分式未變（已逐行比對），可直接沿用。
NDT_PARAMS = {
    "resolution": 1.0,
    "step_size": 0.1,
    "trans_epsilon": 1e-5,
    "max_iterations": 10,
    "converged_param_transform_probability": 1.6,
    "debug": False,
    "base_frame": "base_link",
    "odom_frame": "odom",
    "map_frame": "map",
    "use_gps_as_backup": False,          # 模擬沒有 RTK
    # map->odom 的 TF 前瞻容差。NDT 必須先收到第 N 幀點雲、算完才能發對應 TF，
    # 所以 map->odom 永遠落後點雲約一個週期（實測 0.1 s）。用精確時戳查 TF 的
    # 消費端（RViz 的 PointCloud2 display）會間歇性報 extrapolation into the future。
    # 0.0 = 車端原行為；0.1 讓 TF 的有效窗蓋過一個 NDT 週期。不影響定位運算。
    # 實測 map→odom 最大間隔 149 ms（偶發不收斂時更久），0.1 蓋不住 →
    # RViz 的 MessageFilter（Filter size 10）排隊溢位就丟訊息並報錯。設 0.3 留足裕度。
    "transform_tolerance": 0.3,
}

#: odom 漂移。確定性 UMBmark 模型，同一條軌跡每次跑出完全相同的漂移（論文 A/B 對照需要）。
DRIFT_PARAMS = ["-p", "e_s:=-0.010", "-p", "e_d:=0.0049", "-p", "e_b:=0.0",
                "-p", "output_rate_hz:=20.0", "-p", "zero_start:=true",
                "-p", "publish_tf:=true"]


def _py(script: str, *args, name: str = "", condition=None) -> ExecuteProcess:
    """跑 sim_ws/scripts 下的獨立 rclpy 節點。"""
    return ExecuteProcess(
        cmd=["python3", str(SCRIPTS / script), "--ros-args",
             "-p", "use_sim_time:=true", *args],
        name=name or script.replace(".py", ""),
        output="log",
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    routing_share = Path(get_package_share_directory("campusrover_routing")) / "share" / "node_module"
    desc_share = Path(get_package_share_directory("campusrover_description"))
    ndt_share = Path(get_package_share_directory("ndt_localizer"))

    args = [
        DeclareLaunchArgument("enable_ndt", default_value="true"),
        DeclareLaunchArgument("enable_routing", default_value="true"),
        DeclareLaunchArgument("enable_policy", default_value="true"),
        DeclareLaunchArgument("enable_rviz", default_value="true"),
        # 預設用 .62 帶過來的那份（車端實際在用、分類完整）。
        # sim_deploy_minimal.rviz 是精簡版備援。
        DeclareLaunchArgument("rviz_config", default_value="demo_from_62.rviz"),
        DeclareLaunchArgument("enable_smear", default_value="true",
                              description="點雲掃描運動拖影（NDT 主要誤差源）"),
        DeclareLaunchArgument("enable_drift", default_value="true",
                              description="odom 漂移注入"),
        DeclareLaunchArgument("ndt_crop_min_z", default_value="-1.2",
                              description="NDT 輸入點雲的 z 下限（感測器座標系）。"
                                          "車端預設 -2.0（含地板）；模擬用 -1.2 切掉地板。"),
        DeclareLaunchArgument("spawn_node", default_value="c28",
                              description="NDT 初始猜測要用哪個 routing 節點"),
        # ⚠ checkpoint 與其配套 yaml 必須**整組**切換 —— 這些模型的觀測契約不同
        #   （sa6 是 raw_obs 139 無 action stacking；sa1r1/sa4*/sa5r2 是 raw_obs 83
        #   + frame_stack 8）。只換 model_path 會讓 policy 拿到錯維度的觀測，
        #   **不報錯、只會算出垃圾動作**。故用 profile 綁定，不開放單獨指定模型。
        DeclareLaunchArgument(
            "rl_profile", default_value=S.DEFAULT_RL_PROFILE,
            description="RL checkpoint profile：" + " / ".join(S.RL_PROFILES)),
        DeclareLaunchArgument("initial_mode", default_value="nav",
                              description="policy 起始模式。83D 系列 yaml 預設 idle（實車待命）"),
        DeclareLaunchArgument(
            "speed_rate", default_value="",
            description="覆寫 profile yaml 的 speed_rate；留空＝沿用 yaml 值"),
    ]
    ndt_on = IfCondition(LaunchConfiguration("enable_ndt"))

    # ── 1. URDF：唯一的 base_footprint→base_link→velodyne_link 來源 ──
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="log",
        parameters=[{
            # ⚠ Command() 的輸出必須包 ParameterValue(value_type=str)，
            # 否則 launch 會試著把整份 URDF 當 YAML 解析而失敗。
            "robot_description": ParameterValue(
                Command(["xacro ", str(desc_share / "urdf" / "campusrover_chgh.xacro")]),
                value_type=str),
            "use_sim_time": True,
        }],
        # .62 的 demo.rviz（車端實際在用的那份）的 RobotModel 吃 /charge_description。
        # 讓模擬跟著實車的慣例走，而不是去改使用者慣用的 rviz 設定。
        remappings=[("robot_description", "/charge_description")],
    )

    # ── 2. 感測／底盤降級層（取代實車的 driver 與 velodyne_driver）──
    drift = _py("odom_drift_injector.py",
                "-p", "input_topic:=/odom_gt", "-p", "output_topic:=/odom", *DRIFT_PARAMS,
                name="odom_drift_injector",
                condition=IfCondition(LaunchConfiguration("enable_drift")))
    smear = _py("lidar_motion_smear.py",
                "-p", "input_topic:=/velodyne_points_ideal",
                "-p", "output_topic:=/velodyne_points",
                "-p", "odom_topic:=/odom_gt",          # 拖影由**真值**運動造成
                "-p", "scan_period_s:=0.1",
                # ⚠ 不能用 condition 把整個節點關掉 —— Isaac 發的是 /velodyne_points_ideal，
                #   沒有這個節點就沒人產生 /velodyne_points，NDT 會完全收不到點雲。
                #   enabled:=false 時節點仍在，只是原封不動轉發（A/B 對照用）。
                "-p", ["enabled:=", LaunchConfiguration("enable_smear")],
                name="lidar_motion_smear")

    # ── 3. NDT（與實車同一支程式、同一份參數）──
    ndt = Node(
        package="ndt_localizer", executable="ndt_localizer_node",
        name="ndt_localizer_node", output="screen",
        parameters=[{**NDT_PARAMS, "use_sim_time": True}],
        remappings=[("ndt_pose", "/ndt_pose"), ("diagnostics", "/diagnostics")],
        condition=ndt_on,
    )
    world_to_map = Node(
        package="tf2_ros", executable="static_transform_publisher", name="world_to_map",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"], output="log",
        condition=ndt_on,
    )
    downsample = TimerAction(period=2.0, condition=ndt_on, actions=[Node(
        package="ndt_localizer", executable="voxel_grid_filter_node", name="voxel_grid_filter",
        parameters=[{
            "points_topic": "/velodyne_points", "leaf_size": 0.5,
            "crop_x": 40.0, "crop_y": 40.0, "crop_max_z": 2.0,
            # crop_min_z 是**感測器座標系**。車端預設 -2.0（地板在 -1.43，含在內）。
            # 模擬的 NavFloor 是 152x24 m 的大平面，在真實地圖沒有對應物的方向
            # 也會產生地面回波；NDT 是 6DOF，會轉 roll/pitch 去湊那個大平面
            # （實測 roll -13.1deg / pitch +10.0deg，而車體真值只有 +0.7/+1.4deg）。
            # 設 -1.2 可把地板切掉，只留高於地面 0.23 m 的結構。
            "crop_min_z": ParameterValue(LaunchConfiguration("ndt_crop_min_z"), value_type=float),
            "use_sim_time": True}],
        remappings=[("/points_raw", "/sync_drivers/points_raw")], output="log")])
    map_loader = TimerAction(period=3.0, condition=ndt_on, actions=[Node(
        package="ndt_localizer", executable="map_loader_node", name="map_loader",
        parameters=[{"pcd_path": str(ndt_share / "map" / "3F_314.pcd"), "use_sim_time": True}],
        output="log")])

    # NDT 的初始猜測是 map 原點，c28 在 (-5.67,-0.49) 距原點 5.7 m 超出收斂半徑，
    # 必須餵一次 /initialpose。延後到 map_loader 之後。
    init_pose = TimerAction(period=6.0, condition=ndt_on, actions=[
        _py("publish_initial_pose.py", "-p", ["node_name:=", LaunchConfiguration("spawn_node")],
            name="sim_initial_pose")])

    # ── 4. 地圖與拓撲路徑規劃 ──
    map_server = Node(
        package="campusrover_demo", executable="simple_map_publisher", name="map_server",
        parameters=[{"map_file": str(SIM_WS / "map" / "4v3F.yaml"), "use_sim_time": True}],
        output="log")
    routing_on = IfCondition(LaunchConfiguration("enable_routing"))
    routing_engine = Node(
        package="campusrover_routing", executable="routing_engine_node",
        name="routing_engine_node", output="log", condition=routing_on,
        parameters=[{
            "enable_one_way": False, "use_csv": False, "path_orienation": False,
            "file_path1": str(routing_share / "3F_modul.csv"),
            "file_path2": str(routing_share / "3F_modul.csv"),
            "file_path3": str(routing_share / "3F_modul.csv"),
            "file_node_info": str(routing_share / "3F_info.csv"),
            "connect_method": "common",          # ⚠ 空路徑的 SIGSEGV 防呆在車端未提交的修正裡
            "path_resolution": 0.05, "bezier_length": 1.5, "bezier_resolution": 0.01,
            "BSpline_k": 3, "BSpline_resolution": 0.001, "path_frame": "map",
            "use_sim_time": True,
        }])
    # ⚠ json_folder / json_file 一定要給 —— 預設是空字串，mapinfo 讀不到站名表，
    #   routing service 會回傳空路徑 routing=[]（不會報錯，很難查）。
    #   車端 deploy_full.launch.py:230-234 指定 itc_3f_3.json。
    mapinfo = Node(package="campusrover_routing", executable="mapinfo_db_handler.py",
                   name="mapinfo_db_handler", output="log", condition=routing_on,
                   parameters=[{
                       "use_database": False,
                       "json_folder": str(routing_share.parent / "json") + "/",
                       "json_file": "itc_3f_3.json",
                       "use_sim_time": True,
                   }])
    routes_viz = Node(package="campusrover_routing", executable="routes_visualization",
                      name="routes_visualization_node", output="log", condition=routing_on,
                      parameters=[{"use_sim_time": True}])
    routing_to_path = Node(package="rover_rl_inference", executable="routing_to_path",
                           name="routing_to_path", output="log", condition=routing_on,
                           parameters=[{"building": "itc", "floor": "3",
                     "topic_global_path": "/global_path", "use_sim_time": True}])
    routing_click = Node(package="rover_rl_inference", executable="routing_click_bridge",
                         name="routing_click_bridge", output="log", condition=routing_on,
                         parameters=[{"use_sim_time": True}])

    # ── 5. RL policy（與實車同一支節點）──
    policy_on = IfCondition(LaunchConfiguration("enable_policy"))
    def _rl_nodes(context, *_):
        """依 rl_profile 一次決定 checkpoint + policy yaml + preprocessor yaml。"""
        name = LaunchConfiguration("rl_profile").perform(context)
        if name not in S.RL_PROFILES:
            raise RuntimeError(f"未知的 rl_profile '{name}'，可用：{list(S.RL_PROFILES)}")
        ckpt, policy_yaml, preproc_yaml = S.RL_PROFILES[name]
        model = ROVER_RL / "models" / ckpt
        if not model.exists():
            raise RuntimeError(f"profile '{name}' 的 checkpoint 不存在：{model}")

        overrides = {
            # yaml 的 model_path 指向車端 ~/rover_rl/models/，PC 上在 IsaacLab/rover_rl/
            "model_path": str(model),
            # action contract fixture 同理（policy 啟動時的 manifest 驗證會查它；
            # 缺了會 [FAIL] action_fixture_present 並在 manifest_strict 下直接退出）
            "manifest_fixture_path": str(ROVER_RL / "docs" / "freeze" / "sa1_action_contract_v1.json"),
            # 實車走 lcr_cmd_vel_mux（/input/nav_cmd_vel → /cmd_vel）以避免 joy/nav 互搶。
            # 模擬沒有其他 cmd_vel 來源，直接發給 Isaac 的 ROS2SubscribeTwist。
            "topic_cmd_vel": "/cmd_vel",
            # 83D 系列的 yaml 預設 initial_mode="idle"（實車首次上電待命，人確認後才切 nav）。
            # 模擬沒有人在旁邊按確認，直接起 nav。
            "initial_mode": LaunchConfiguration("initial_mode").perform(context),
            "use_sim_time": True,
        }
        sr = LaunchConfiguration("speed_rate").perform(context).strip()
        if sr:
            overrides["speed_rate"] = float(sr)

        print(f"[sim_deploy] RL profile '{name}': {ckpt}")
        print(f"[sim_deploy]   policy yaml     = {policy_yaml}")
        print(f"[sim_deploy]   preprocess yaml = {preproc_yaml}")
        return [
            Node(package="rover_rl_inference", executable="lidar_preprocessor",
                 name="rover_rl_lidar_preprocessor", output="log", condition=policy_on,
                 parameters=[str(RL_CFG / preproc_yaml), {"use_sim_time": True}],
                 additional_env=_rl_env()),
            Node(package="rover_rl_inference", executable="policy_node",
                 name="rover_rl_policy", output="screen", condition=policy_on,
                 parameters=[str(RL_CFG / policy_yaml), overrides],
                 additional_env=_rl_env()),
        ]

    rl_nodes = OpaqueFunction(function=_rl_nodes)

    rviz = Node(package="rviz2", executable="rviz2", name="rviz_sim", output="log",
                arguments=["-d", [str(SIM_WS / "rviz") + "/", LaunchConfiguration("rviz_config")]],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("enable_rviz")))

    return LaunchDescription([
        *args, robot_state_publisher, drift, smear,
        world_to_map, ndt, downsample, map_loader, init_pose,
        map_server, routing_engine, mapinfo, routes_viz, routing_to_path, routing_click,
        rl_nodes, rviz,
    ])
