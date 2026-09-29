# 交接：Isaac Sim 走廊模擬錄影（2026-09-29）

> 給接手的 AI／人。先讀這份，再依「要讀的記憶」順序讀。

## 1. 工作區

| 項目 | 位置 |
|---|---|
| 工作目錄 | `/home/aa/IsaacLab/sim_ws`（git repo `isaac_rover_rl`，branch `main`，remote `git@github.com:me0608623/isaac_rover_rl.git`） |
| 上層專案 | `/home/aa/IsaacLab`（CLAUDE.md 在這裡，branch `wdclean-repro-20260429-pcB`） |
| 錄影結果 | `sim_ws/recordings/`（72 趟，git 不追蹤影片）；中文索引 `recordings/00_影片總覽/` |
| 報告 | `sim_ws/reports/`（`driving_time.md/.csv`、`localization_error.txt`、`recordings_health.txt`） |
| 本交接包 | `sim_ws/docs/handoff_20260929/`：`論文素材包/`（要給 Windows 的文件）、`手機網頁/`（artifact 原始檔）、`scripts_tmp/`（一次性批次腳本） |

## 2. 要讀的記憶（依序）

1. `/home/aa/IsaacLab/CLAUDE.md`：專案規則（繁中註解、不用 `conda run`、Obsidian 優先）
2. `/home/aa/.claude/projects/-home-aa-IsaacLab/memory/MEMORY.md`：記憶索引，★★★ 必讀；本工作相關：
   - `finding_record_batch_72run_pitfalls_20260924.md`（本輪七個坑：dynamic 自殺、例外被吞、計時錯誤、Character_04、相對路徑、鏡頭穿牆…）
   - `finding_nearest_obstacle_source_decomposition_20260923.md`、`finding_velodyne_is_physx_lidar_20260923.md`、`finding_sim_robot_collision_model_20260923.md`
   - `project_orca_pedestrians_20260922.md`、`finding_sim_wheel_radius_11pct_fast_20260922.md`、`finding_isaac_headless_render_black_20260922.md`
   - `feedback_pkill_dash_f_self_match.md`（`pkill -f` 會殺到自己）、`feedback_thesis_review_folder_two_machines.md`（Windows 傳檔）
3. `sim_ws/docs/handoff_20260929/論文素材包/01_模擬作法.md`：模擬怎麼做（USD、行人、障礙、錄影、指標、限制）
4. `sim_ws/recordings/README.md`：逐趟總表與目錄結構
5. Obsidian：`/home/aa/Documents/Obsidian Vault/isaaclab/2026-09-23_論文錄影_ORCA行人與最近障礙來源分解.md`
6. 完整對話紀錄（需要細節時）：`/home/aa/.claude/projects/-home-aa-IsaacLab/c91a54e7-f2a8-4332-9178-234b00ffefba.jsonl`

## 3. 目前狀態

- 72 趟（3 模型 × 2 路線 × 3 情境 × 4 趟）全部來回抵達、0 碰撞；影片 288 支（每趟 4 支，含三視角合成）
- 已修：Character_04 身體偏移、鏡頭穿牆（斜前方拉近、車後橫移）、行人轉身平滑、計時 /clock 競態、dynamic 停用角色崩潰
- 最新 commit：`a112bf5`（三視角合成加即時線速度／角速度曲線）

## 4. 進行中（2026-09-29 14:45 更新）

| 工作 | 狀態 | 接手要做的事 |
|---|---|---|
| 三視角合成（含速度曲線）72 支 | ✅ 完成，已傳到 Windows 並以大小比對 72/72 一致 | — |
| 影片到 Windows | ✅ 288 支都在 `…\isaacsim 走廊模擬\影片\` | — |
| 說明文件 | ⛔ **暫停**：只傳了 `00_先看這裡.md`（舊版） | 等使用者同意後，傳 `論文素材包/` 的 00~03、`結果/`、`場景/`（共約 1 MB） |
| 手機網頁 | 已發佈 v5 | 可換新版 `v/sa4r2_c27_mixed4_combined.mp4` 再發佈 |

⚠⚠ **傳到 Windows 會算進學校每日 15 GB 流量**（雖然走內網）。09-29 傳 17 GB 已讓 140.124.42.62 被封到隔天 6 點。
任何大檔傳輸前先問使用者（見記憶 `feedback_lan_transfer_counts_campus_quota.md`）。

## 4.5 下一個任務（使用者指定）

**提高靜態與動態障礙的數量與密度，做壓力測試** —— 規格見同資料夾
`TASK_壓力測試_提高障礙密度.md`。重點：c27 路線在現有間距規則下最多只能擺 6 個靜態障礙
（已用滿），要更密必須放寬間距並加可通行檢查；角色只有 20 個，行人要加倍得先新增角色。

## 5. scp 到 Windows 的注意事項

- 路徑**不要**在遠端路徑內再加一層引號（新版 scp 走 SFTP，會把引號當檔名）：
  `scp 檔案 'aa@192.168.0.62:C:/Users/aa/Documents/論文撰寫/video/isaacsim 走廊模擬/'`
- 遠端查檔用 `powershell -NoProfile -Command "[Console]::OutputEncoding=[Text.Encoding]::UTF8; ..."` 才不亂碼

## 6. 常用指令

```bash
cd /home/aa/IsaacLab/sim_ws
source ./setup_sim_env.sh
# 單趟重錄（導航＋回放）
ONLY=sa4r2_c27_mixed_run04 bash scripts/record_batch.sh
# 行駛時間表（修正計時錯誤）
python3 scripts/driving_time.py recordings > reports/driving_time.md
# 三視角合成
python3 scripts/make_combined.py recordings
# 測試（ROS 的 pytest plugin 會衝突，要關掉）
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest scripts/ -q -p no:cacheprovider
```

## 7. 使用者偏好

- 回覆用繁體中文、簡短、多用表格；聽不懂時要用白話重講
- GPU 與他人共用（laksh、cm），開大批次前先看 `nvidia-smi`
- 舊錄影一律保留不刪（`recordings/_舊版與作廢錄影_保留不刪/`）
- 要求批次期間每 5 分鐘回報
