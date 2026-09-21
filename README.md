# isaac_rover_rl

在 **Isaac Sim** 的校園 3F 走廊場景裡，跑**與實車完全相同的** ROS 2 導航堆疊
（`ndt_localizer` + `campusrover_routing` + `rover_rl` RL policy），
用於部署前的 sim-in-the-loop 驗證。

```
實車： campusrover_driver + velodyne_driver           → /odom, /velodyne_points
模擬： Isaac Sim + odom_drift_injector + motion_smear  → /odom, /velodyne_points
```

**NDT / routing / policy 是同一套節點、同一份參數。** 兩邊的唯一差異是誰提供感測與底盤。

---

## 快速開始

```bash
# 終端機 1 — 模擬器
cd sim_ws && ./run_sim.sh              # headless（RTF ≈ 0.9）
                                       # --gui 可開視窗，但 RTF 掉到 0.02

# 終端機 2 — ROS 堆疊 + RViz
cd sim_ws && ./sim_deploy_shell.sh     # 背景起棧 + 前景 TUI（需真實 TTY）
#            ./sim_deploy.sh           # 前景 log 版，非互動環境用
#            ./sim_deploy_stop.sh      # 收棧（不動 Isaac）
```

RViz 裡可直接操作：**2D Goal Pose** 給 policy 目標、**Publish Point** 點兩下規劃 routing 路徑、
**2D Pose Estimate** 在 NDT 跑掉時重新定位。

```bash
./run_tests.sh          # 72 項測試，不需要 Isaac Sim 也不需要 ROS
```

---

## 前置作業（首次）

這個 repo **只包含自己寫的程式與文件**。車端來的套件、模型、地圖都靠 rsync 取得 ——
車端交接單明確要求「一律 rsync 不要 clone」：`ndt_localizer` 不在任何 git repo，
`rover2_ws` 本地領先 origin 且帶有關鍵的未提交修正。

```bash
# 1. 車端 ROS 套件 → src/
rsync -a --exclude 'map/0F*.pcd' --exclude 'map/1F.pcd' --exclude 'map/3F.pcd' \
  aa@<車IP>:/home/aa/ndt_ws/src/ndt_localizer src/
rsync -a --exclude build --exclude install --exclude .git \
  aa@<車IP>:/home/aa/rover2_ws/src/campusrover_routing \
  aa@<車IP>:/home/aa/rover2_ws/src/campusrover_base/campusrover_msgs \
  aa@<車IP>:/home/aa/rover2_ws/src/campusrover_base/campusrover_description \
  aa@<車IP>:/home/aa/rover2_ws/src/campusrover_system/campusrover_demo  src/

# 2. 柵格地圖
rsync -a aa@<車IP>:/home/aa/maps/4v3F.{yaml,pgm} map/

# 3. 建置（ROS 2 Jazzy）
./build.sh

# 4. PyTorch（系統 python3.12 沒有；與車端同版）
pip install --target vendor/py312 --index-url https://download.pytorch.org/whl/cpu torch==2.10.0

# 5. 測試用 venv（與 conda env_isaaclab 隔離，避免蓋掉 Isaac Sim 自帶的 pxr）
python3 -m venv .venv && .venv/bin/pip install usd-core numpy scipy pytest

# 6. 產生修正後的 USD 疊加層
python3 scripts/build_ros_graph.py --obstacles
```

`src/` 相對車端的必要修改記在 [`patches/README.md`](patches/README.md)，每一項都可稽核。

---

## 設計要點

### USD 用疊加層，原始檔零改動

`scripts/build_ros_graph.py` 讀原始 USD，把所有修正寫成一份 **ASCII `.usda` 疊加層**
（`assets/3floor_ver_1_ros_fixed.usda`）。原始檔一個位元都不動，疊加層可以 `git diff`。

純 `usd-core` 實作，**不需要開 Isaac Sim** —— OmniGraph 節點在 USD 裡只是帶 `node:type`
的普通 prim，可以直接 author。

### 規格層與套用層分離

`scripts/ros_graph_spec.py` 是**單一事實來源**：所有「必須與實車一致」的數字
（輪距 0.559212、`base_footprint→velodyne_link` 1.43 m、World↔map 配準…）都在這裡，
純 Python 無 Isaac 相依，72 項測試在 0.3 秒內跑完。

### TF 權責（方案 A）

Isaac **一條 TF 都不發**：

```
map            → odom             ndt_localizer
odom           → base_footprint   odom_drift_injector（publish_tf:=true）
base_footprint → base_link → …    robot_state_publisher（實車 URDF）
```

TF 數字全部來自實車 URDF，不受 USD 幾何誤差污染。

### 降級層：模擬出理想值，降級節點注入真實世界的缺陷

```
Isaac /odom_gt               ──odom_drift_injector──▶ /odom
Isaac /velodyne_points_ideal ──lidar_motion_smear ──▶ /velodyne_points
```

### RL checkpoint 用 profile 綁定

checkpoint 與其配套的 `policy_params_*` / `lidar_preprocessor_params_*` **只能整組切換** ——
這些模型的觀測契約不同（sa6 是 raw_obs 139 無 action stacking；sa1r1/sa4*/sa5r2 是
raw_obs 83 + frame_stack 8）。只換 `model_path` 會讓 policy 拿到錯維度的觀測，
**不報錯、只會安靜地算出垃圾動作**。

```bash
ros2 launch launch/sim_deploy.launch.py rl_profile:=sa4r2   # 預設
#                                       rl_profile:=sa4r3 / sa5r2 / sa1r1 / sa6
```

---

## 環境需求

| | 版本 |
|---|---|
| OS | Ubuntu 24.04 |
| ROS | **Jazzy**（車端是 Humble，C++ 套件需重編） |
| Isaac Sim | 5.1（conda `env_isaaclab`，Python 3.11） |
| PyTorch | 2.10.0+cpu（與車端同版） |

⚠ **RMW 必須統一 `rmw_fastrtps_cpp`**（`setup_sim_env.sh` 會設）。
Isaac Sim 的 ROS 2 bridge 內建的 rmw **只有 FastDDS**，沒有 CycloneDDS。
RMW 不同的節點**完全看不到彼此，而且不會報任何錯**。

---

## 文件

[`docs/2026-09-21_模擬ROS契約對照_PC端回覆.md`](docs/2026-09-21_模擬ROS契約對照_PC端回覆.md)
—— 與車端交接單配對的完整技術記錄：模擬與實車的逐項契約對照、
World↔map 配準、實機啟動驗證、以及過程中踩到並修正的所有陷阱。
