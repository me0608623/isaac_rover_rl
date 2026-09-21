# sim_ws/src 相對於車端的必要修改

`src/` 是從車端（Ubuntu 22.04 / ROS 2 Humble）rsync 過來的，
在 PC 端（Ubuntu 24.04 / ROS 2 Jazzy）需要少量移植修改。

**原則：只改「不改就跑不起來」的東西，不改演算法、不改參數。**
每一項都記在這裡，讓 sim 與實車的差異隨時可稽核。

| # | 檔案 | 修改 | 原因 |
|---|---|---|---|
| 1 | `campusrover_demo/scripts/simple_map_publisher` | shebang `#!/usr/bin/python3.10` → `#!/usr/bin/env python3` | Ubuntu 24.04 沒有 `python3.10`。OS 回報的 `No such file or directory` 指的是**直譯器**不存在，不是腳本不存在 —— 訊息極易誤導 |
| 2 | `ndt_localizer/nodes/ndt.cpp` + `include/ndt.hpp` | 新增 `transform_tolerance` 參數（**預設 0.0 = 原行為**），`publish_tf` 把 map→odom 的時戳加上該容差 | NDT 必須先收到第 N 幀點雲、算完才能發對應 TF，所以 `map→odom` 永遠落後點雲約一個週期（實測：點雲時戳 188.000、map→odom 187.900）。用**點雲精確時戳**查 TF 的消費端（RViz 的 PointCloud2 display）會間歇性報 `Lookup would require extrapolation into the future` —— 畫面上就是點雲閃爍。AMCL / slam_toolbox 等標準定位節點都用同樣機制。**只延長 TF 有效時間窗，不動任何定位運算**；`sim_deploy.launch.py` 設 0.1，車端維持 0.0 即與原版完全相同 |

## 未修改但已查證無影響

| 項目 | 結論 |
|---|---|
| PCL 1.12（車端）→ 1.14（PC） | `getTransformationProbability` 只是改名為 `getTransformationLikelihood`，計算式 `score / input_->size()` 與 `gauss_d1_/d2_/d3`、`outlier_ratio_=0.55` 逐字元相同 → **收斂門檻 1.6 可直接沿用** |
| `ndt.cpp` TF 時戳修正 | 617 行版本，`lookupTransform(…, time_stamp)` @550 已包含 |
| `node_connection.cpp` 空陣列防呆 | 225 行版本，`if (nodes.empty()) continue;` @54 已包含 |
