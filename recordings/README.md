# 論文錄影產物

Isaac Sim 裡跑**與實車同一套 ROS stack**（ndt_localizer + campusrover_routing
+ rover_rl policy + vo_safety_node）的 3F 走廊來回導航。

**3 模型 × 3 情境 × 4 難度 × 3 視角 = 108 段影片**，每趟附一份命名同步的 rosbag。

## 要挑影片看 → 找「中文影片」資料夾

裡面是**中文命名的符號連結**，檔名直接寫清楚是哪個模型、什麼情境、
第幾趟、場上有幾個靜態物幾個動態物、哪個機位。

兩個地方都有，內容一樣（同一支程式產生，不會走樣）：

```
recordings/模型sa4r2/00_中文影片/    ← 點進某個模型時就在眼前
recordings/00_影片總覽/              ← 三個模型 + 三組對照，一次看全
```

```
00_影片總覽/
    01_主批次_三模型正式錄影/模型sa5r2/混合_ORCA互動_第4趟_靜6動8_俯視.mp4
    02_對照_行人走固定路線_非ORCA/   03_對照_速度0.6/   04_對照_速度1.0/
    05_每趟原始資料_bag與CSV/模型sa5r2_混合_ORCA互動_第4趟_靜6動8/
```

索引裡的檔名格式：**`{情境}_{行人走法}_第N趟_靜{靜態數}動{動態數}_{機位}`**

| 欄位 | 值 | 原文 | 意思 |
|---|---|---|---|
| 情境 | 純靜態 | `static` | 場上沒有任何會動的東西 |
| | 純動態 | `dynamic` | 障礙圓柱全關，但**站立人物仍在場**（見下） |
| | 混合 | `mixed` | 障礙 + 走動行人都有 |
| 行人走法 | ORCA互動 | `crowd_mode=orca` | RVO2，會閃避彼此**與車** |
| | 固定路線 | `crowd_mode=path` | 沿固定路線來回，不理車（只有對照1）|
| | 不走動 | — | 沒有人在走（純靜態）|
| 機位 | 俯視 / 車後 / 斜前方 | `topdown` / `chase` / `oblique` | 三個相機 |

⚠ **`crowd_mode` 不能直接當標籤。** `static` 那幾趟的 `run.json` 照樣記
`crowd_mode: orca`（那只是 CLI 參數），但 `walks_enabled=False`、實際沒有人
在走。所以標籤看**實際動態數**：動態為 0 就標「不走動」。

### 靜/動數量是怎麼數的

一律從**該趟自己的 `isaac_nav.log`** 數（`scripts/scene_counts.py`），
不是查 `scene_variants` 的計畫表 —— 執行期會停用太靠近車、站在 routing
點上、或擠在一起的角色，拿計畫值寫進檔名等於讓檔名有機會說謊。

```
靜態數 = 啟用的障礙圓柱/箱 + 不會動的角色 − 重疊（站立人物疊在 person 障礙上）
動態數 = 沿路徑移動的角色
```

所以同一個「第4趟」在三個情境的實際密度差很多：

| 情境 | 第1趟 | 第2趟 | 第3趟 | 第4趟 |
|---|---|---|---|---|
| 純靜態 | 靜5動0 | 靜8動0 | 靜11動0 | **靜14動0** |
| 純動態 | 靜3動2 | 靜4動4 | 靜4動6 | 靜5動8 |
| 混合 | 靜3動2 | 靜4動4 | 靜5動6 | 靜6動8 |

**純靜態才是靜態物最密的情境**（走動人物停在原位也算靜態，而且
Character_10~13、19 的原位就在走廊中線上）—— 這就是 static 那幾趟
耗時最長、最近距離最差的原因。

重建索引（重錄後跑一次就好，只建連結、不複製檔案）：

```bash
python3 scripts/make_browse_tree.py
```

⚠ **真資料夾名（`sa4r2_mixed_run04`）刻意不改成中文。** tag 被寫進
`run.json` 的 `tag` 欄、`nav/<tag>_leg?_*.csv`、`video/<tag>_*.mp4`，而
`record_batch.sh ONLY=<tag>` 與所有分析程式都靠它對應；改名會讓「重跑某一趟」
和全部分析失效，而且重跑批次又會把舊名字生回來。

## 場景

| 情境 | 障礙圓柱/箱 | 站立人物 | 走動人物 |
|---|---|---|---|
| `static` | 啟用 | 在障礙位置 | **站在 USD 原位、不走動** |
| `dynamic` | 全部關閉 | **仍在場** | ORCA 互動避讓 |
| `mixed` | 啟用 | 在障礙位置 | ORCA 互動避讓 |

⚠ 兩件容易誤會、2026-09-23 逐幀查證過的事（寫論文敘述時要照這個）：

- `dynamic` 的 log 印「靜態障礙 0/18 啟用」**只關掉圓柱**，
  `place_standing` 擺的站立人物還在場上，光達照樣打得到。
- `static` 的走動人物不是消失，而是**停在 USD 原始位置**；
  其中 Character_10~13、19 的原位剛好就在**走廊中線上**——
  這就是 static 那幾趟耗時最長、最近距離最差的原因。

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

重算：`PYTHONPATH= .venv/bin/python scripts/nearest_source.py recordings`
（完整報表在 `reports/nearest_source.txt`）

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
recordings/
    00_影片總覽/               中文命名的符號連結索引（見上面）
    模型sa4r2/  模型sa4r3/  模型sa5r2/      ← 先按模型分類
        00_中文影片/           該模型 36 段的中文連結
        <模型>_<情境>_run<NN>/ 各 12 趟的原始資料，內容見下
    _作廢批次_只留紀錄/       兩份被取代的舊批次，影片與 bag 已刪（省 20 G），
                              只留 log 與 run.json；數字**勿引用**：
                              v1 輪徑未修正、車速偏快 11%；v2 行人還走直線未用 ORCA
    batch.log  README.md

recordings_abl/               三組對照（各 12 趟，單一模型 sa4r2 故無模型層）
    對照1_行人走固定路線_非ORCA/   ← 唯一不用 ORCA 的一組
    對照2_速度0.6/                 ← 仍是 ORCA，只改車速上限
    對照3_速度1.0/                 ← 仍是 ORCA，只改車速上限
```

⚠ 「什麼算一趟」的唯一定義在 `scripts/run_layout.py` 的 `run_dirs()` ——
它會同時看 root 底下與模型子資料夾底下兩層，並排除索引樹裡的符號連結
（不排除的話每趟會被算兩次）。**新增分析程式請用它，不要自己列目錄。**

```
recordings/模型<模型>/<模型>_<情境>_run<NN>/
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
python3 scripts/make_browse_tree.py                   # 重建中文索引
python3 scripts/make_readme.py recordings > recordings/README.md
```

⚠ 這份 README 是 `make_readme.py` 產生的 —— **不要手改**，改了下次重新產生
就會被蓋掉。要改內容請改 `scripts/make_readme.py` 的 `render()`。

⚠ 一趟要跑**兩遍**（見 `scripts/pose_log.py` 檔頭）：path tracing 會把
RTF 壓到 0.35，邊導航邊算圖時 cmd_vel 被釘在 0.060 m/s（正常 0.475），
到不了終點。所以第一遍全速導航＋錄 bag＋寫位姿，第二遍照位姿回放算圖。
