# 論文錄影產物

Isaac Sim 裡跑**與實車同一套 ROS stack**（ndt_localizer + campusrover_routing
+ rover_rl policy + vo_safety_node）的 3F 走廊來回導航。

**3 模型 × 3 情境 × 4 難度 × 3 視角 = 108 段影片**，每趟附一份命名同步的 rosbag。

## 場景

| 情境 | 靜態障礙 | 行人 |
|---|---|---|
| `static` | 有 | 站著（不走動）|
| `dynamic` | 無 | ORCA 互動避讓 |
| `mixed` | 有 | ORCA 互動避讓 |

每趟（run01→run04）的障礙與行人**數量遞增、位置各不相同**：
3/4/5/6 個障礙、2/4/6/8 個走動行人。走廊中段固定有**兩人肩並肩站在一側**，
逼車走另一邊；其餘障礙沿走廊左右交錯。人形障礙是真的虛擬人物
（不可見的圓柱負責物理碰撞，人物負責外觀與光達輪廓）。

## 結果

碰撞幀 = LiDAR 最近距離 ≤0.45 m 的取樣點數（車體半徑 0.35 + 緩衝 0.10），
**不是**物理碰撞事件。

### 「最近障礙」到底是靠近了什麼

碰撞幀的數字若有一部分其實是**牆**，這個指標就不能當論文的避障證據 ——
走廊很窄，貼著牆走是正常的。所以逐幀把最近距離拆成四個來源
（`scripts/nearest_source.py`：車的真值位姿 + 行人軌跡 + 該趟障礙表 +
佔據圖距離場，對齊後與 nav CSV 實測的 `min_range_m` 逐點核對）。

36 趟 / 33092 個取樣點，「最近的東西」是誰：

| 來源 | 佔比 |
|---|---|
| 站立行人（含人形障礙）| 44.1% |
| 牆 | 29.1% |
| 走動行人 | 22.5% |
| 箱型障礙 | 4.3% |

**11 個碰撞幀（實測 ≤0.45 m）的來源：站立行人 6、箱型障礙 1、來源不明 4 ——
沒有一個是牆。** 碰撞幀的數字沒有被牆污染。

⚠ 那 4 個「來源不明」全在 `sa4r3_mixed_run04`：實測 0.447 m，但場上最近的
已知物體在 1.75 m 外。把那片回波用 bag 的 TF 搬到 map 之後（同一片雲有
65% 的點落在佔據圖的牆上 0.05 m 內，所以 TF 沒問題），它落在離最近牆
2.25 m 的空地、車正後方 0.45 m，**隨車一起移動**；佔據圖、建物 Mesh、
18 個障礙 prim、13 個角色的位置全都對不上。最可能是車體自身結構被自己的
RTX 光達打到（RTX 光達打的是算圖網格，不是物理碰撞體）。
**引用碰撞幀時，`sa4r3` mixed 那 4 幀應當排除（該列實為 0）。**

方法本身的誤差：預測−實測的逐趟中位都落在 −0.22 ~ 0.00 m。p05 到 −0.65 m ——
那是佔據圖上有實機掃描留下的雜物、模擬場景裡沒有，所以「離牆距離」偏小。
這只會**高估**牆的佔比，不影響「沒有一個碰撞幀是牆」的結論。

### 依模型 × 情境彙總

| 模型 | 情境 | 抵達 | 最近障礙(最差) | 碰撞幀 | 平均每段耗時 |
|---|---|---|---|---|---|
| `sa4r2` | static | 8/8 段 | 0.45 m | 0 | 63.0 s |
| `sa4r2` | dynamic | 8/8 段 | 0.62 m | 0 | 39.7 s |
| `sa4r2` | mixed | 8/8 段 | 0.55 m | 0 | 38.9 s |
| `sa4r3` | static | 7/8 段 | 0.42 m | 7 | 76.4 s |
| `sa4r3` | dynamic | 8/8 段 | 0.65 m | 0 | 49.3 s |
| `sa4r3` | mixed | 8/8 段 | 0.45 m | 4 | 54.7 s |
| `sa5r2` | static | 7/8 段 | 0.46 m | 0 | 77.4 s |
| `sa5r2` | dynamic | 8/8 段 | 0.52 m | 0 | 72.0 s |
| `sa5r2` | mixed | 8/8 段 | 0.47 m | 0 | 78.7 s |

### 逐趟明細

| tag | 模型 | 情境 | 難度 | 抵達 | 最近障礙 | 碰撞幀 | 影片 |
|---|---|---|---|---|---|---|---|
| `sa4r2_dynamic_run01` | sa4r2 | dynamic | run01 | 2/2 | 0.87 m | 0 | 3 |
| `sa4r2_dynamic_run02` | sa4r2 | dynamic | run02 | 2/2 | 0.64 m | 0 | 3 |
| `sa4r2_dynamic_run03` | sa4r2 | dynamic | run03 | 2/2 | 0.73 m | 0 | 3 |
| `sa4r2_dynamic_run04` | sa4r2 | dynamic | run04 | 2/2 | 0.62 m | 0 | 3 |
| `sa4r2_mixed_run01` | sa4r2 | mixed | run01 | 2/2 | 0.88 m | 0 | 3 |
| `sa4r2_mixed_run02` | sa4r2 | mixed | run02 | 2/2 | 0.72 m | 0 | 3 |
| `sa4r2_mixed_run03` | sa4r2 | mixed | run03 | 2/2 | 0.55 m | 0 | 3 |
| `sa4r2_mixed_run04` | sa4r2 | mixed | run04 | 2/2 | 0.60 m | 0 | 3 |
| `sa4r2_static_run01` | sa4r2 | static | run01 | 2/2 | 0.82 m | 0 | 3 |
| `sa4r2_static_run02` | sa4r2 | static | run02 | 2/2 | 0.68 m | 0 | 3 |
| `sa4r2_static_run03` | sa4r2 | static | run03 | 2/2 | 0.45 m | 0 | 3 |
| `sa4r2_static_run04` | sa4r2 | static | run04 | 2/2 | 0.47 m | 0 | 3 |
| `sa4r3_dynamic_run01` | sa4r3 | dynamic | run01 | 2/2 | 0.82 m | 0 | 3 |
| `sa4r3_dynamic_run02` | sa4r3 | dynamic | run02 | 2/2 | 0.65 m | 0 | 3 |
| `sa4r3_dynamic_run03` | sa4r3 | dynamic | run03 | 2/2 | 0.70 m | 0 | 3 |
| `sa4r3_dynamic_run04` | sa4r3 | dynamic | run04 | 2/2 | 0.71 m | 0 | 3 |
| `sa4r3_mixed_run01` | sa4r3 | mixed | run01 | 2/2 | 1.07 m | 0 | 3 |
| `sa4r3_mixed_run02` | sa4r3 | mixed | run02 | 2/2 | 0.84 m | 0 | 3 |
| `sa4r3_mixed_run03` | sa4r3 | mixed | run03 | 2/2 | 0.70 m | 0 | 3 |
| `sa4r3_mixed_run04` | sa4r3 | mixed | run04 | 2/2 | 0.45 m | 4 | 3 |
| `sa4r3_static_run01` | sa4r3 | static | run01 | 2/2 | 0.89 m | 0 | 3 |
| `sa4r3_static_run02` | sa4r3 | static | run02 | 2/2 | 0.64 m | 0 | 3 |
| `sa4r3_static_run03` | sa4r3 | static | run03 | 1/2 | 0.42 m | 1 | 3 |
| `sa4r3_static_run04` | sa4r3 | static | run04 | 2/2 | 0.45 m | 6 | 3 |
| `sa5r2_dynamic_run01` | sa5r2 | dynamic | run01 | 2/2 | 1.08 m | 0 | 3 |
| `sa5r2_dynamic_run02` | sa5r2 | dynamic | run02 | 2/2 | 0.74 m | 0 | 3 |
| `sa5r2_dynamic_run03` | sa5r2 | dynamic | run03 | 2/2 | 0.52 m | 0 | 3 |
| `sa5r2_dynamic_run04` | sa5r2 | dynamic | run04 | 2/2 | 0.55 m | 0 | 3 |
| `sa5r2_mixed_run01` | sa5r2 | mixed | run01 | 2/2 | 1.25 m | 0 | 3 |
| `sa5r2_mixed_run02` | sa5r2 | mixed | run02 | 2/2 | 0.74 m | 0 | 3 |
| `sa5r2_mixed_run03` | sa5r2 | mixed | run03 | 2/2 | 0.47 m | 0 | 3 |
| `sa5r2_mixed_run04` | sa5r2 | mixed | run04 | 2/2 | 0.73 m | 0 | 3 |
| `sa5r2_static_run01` | sa5r2 | static | run01 | 2/2 | 0.87 m | 0 | 3 |
| `sa5r2_static_run02` | sa5r2 | static | run02 | 2/2 | 0.58 m | 0 | 3 |
| `sa5r2_static_run03` | sa5r2 | static | run03 | 2/2 | 0.51 m | 0 | 3 |
| `sa5r2_static_run04` | sa5r2 | static | run04 | 1/2 | 0.46 m | 0 | 3 |

## 目錄長相

```
recordings/<模型>_<情境>_run<NN>/
    video/<tag>_{topdown,chase,oblique}.mp4
    bag/<tag>/                 ros2 bag（mcap），含 RViz 需要的全部 topic
    nav/<tag>_leg{1,2}_*.csv   逐時刻：位置 / VO / 最近障礙 / 命令速度
    pose.csv / crowd.csv       車與行人的世界位姿（第二遍回放用）
    frame_times.csv            幀號 ↔ 模擬時間
    run.json                   參數、導航結果、影片↔bag 對齊偏移
```

## 影片與 rosbag 怎麼對齊

```bash
ros2 bag play recordings/<tag>/bag/<tag> --clock \
    --start-offset $(jq -r .sync.bag_play_start_offset_s recordings/<tag>/run.json)
```
影片第 N 幀 = 上面的 offset 再加 N/30 秒。

## 怎麼重做

```bash
bash scripts/record_batch.sh                    # 整批（已完成的自動跳過）
ONLY=sa4r2_mixed_run02 bash scripts/record_batch.sh   # 只重跑某一趟
```

⚠ 一趟要跑**兩遍**（見 `scripts/pose_log.py` 檔頭）：path tracing 會把
RTF 壓到 0.35，邊導航邊算圖時 cmd_vel 被釘在 0.060 m/s（正常 0.475），
到不了終點。所以第一遍全速導航＋錄 bag＋寫位姿，第二遍照位姿回放算圖。
