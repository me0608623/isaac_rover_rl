#!/usr/bin/env python3
"""從各趟的 run.json / nav.log 產生結果 README。

用法：  python3 scripts/make_readme.py recordings > recordings/README.md
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

from run_layout import BROWSE_DIRNAME, run_dirs
from scene_counts import counts_from_log

_ROW = re.compile(
    r"^(c\d+)→(c\d+)\s+(OK|FAIL)\s+([\d.]+)s\s+([\d.]+)m\s+([\d.]+)m\s+(\d+)/(\d+)")


def parse_nav_table(text: str):
    """讀 nav.log 末尾的結果表。解析不到就回空。

    ⚠ 不要在解析失敗時回一筆零值 —— 那會讓失敗的趟在彙總表裡長得像成功。
    """
    out = []
    for line in text.splitlines():
        m = _ROW.match(line.strip())
        if not m:
            continue
        out.append({
            "from": m.group(1), "to": m.group(2), "result": m.group(3),
            "seconds": float(m.group(4)), "path_m": float(m.group(5)),
            "min_range_m": float(m.group(6)),
            "collision_frames": int(m.group(7)), "samples": int(m.group(8)),
        })
    return out


def cell_summary(legs):
    """一格（同一個模型/情境/難度）的彙總。最近障礙取**最差**，碰撞幀加總。"""
    if not legs:
        return {"legs": 0, "arrived": 0, "min_range_m": None,
                "collision_frames": 0, "seconds": None}
    return {
        "legs": len(legs),
        "arrived": sum(1 for l in legs if l["result"] == "OK"),
        "min_range_m": min(l["min_range_m"] for l in legs),
        "collision_frames": sum(l["collision_frames"] for l in legs),
        "seconds": sum(l["seconds"] for l in legs) / len(legs),
    }


SCEN_ZH = {"static": "純靜態", "dynamic": "動態", "mixed": "混合"}


def collision_cell(col) -> str:
    """run.json["collisions"] → 表格的一格。

    ⚠ 偵測器沒在工作時**不可以寫 0** —— 那是最危險的輸出，看起來很乾淨。
    """
    if not col:
        return "—"
    d = col.get("detector", {})
    if not d.get("overlap_ok"):
        return "⚠ 偵測器失效"
    n = col.get("episodes", 0)
    if n == 0:
        return "0"
    parts = [f"{k}{v['episodes']}" for k, v in sorted(col.get("by_category", {}).items())]
    return f"**{n}**（" + "、".join(parts) + "）"


def route_of(r) -> str:
    """run.json → 路線 key。舊錄影沒有 route_key，從 route 的終點推。"""
    if r.get("route_key"):
        return r["route_key"]
    rt = r.get("route") or []
    return rt[1] if len(rt) >= 2 else ""


def density_table(runs):
    """路線 × 情境 × 趟次的「靜N動M」表。數字取自各趟 log（run["counts"]）。

    同一格有好幾個模型時，數字應該一樣（場景只依路線、run_index 與情境決定）；
    不一樣就**全部列出**，不要挑一個 —— 那代表執行期停用的角色不同。
    兩條路線的障礙擺法不同（各自只在自己會開到的那段擺），所以要分開列。
    """
    idxs = sorted({r.get("run_index", 0) for r in runs if r.get("run_index")})
    routes = sorted({route_of(r) for r in runs})
    multi = len(routes) > 1 or routes != [""]
    head = ("| 路線 | 情境 | " if multi else "| 情境 | ")
    out = [head + " | ".join(f"第{i}趟" for i in idxs) + " |",
           ("|---|---|" if multi else "|---|") + "---|" * len(idxs)]
    for rk in routes:
        for sc in ("static", "dynamic", "mixed"):
            cells = []
            for i in idxs:
                got = sorted({tuple(r["counts"]) for r in runs
                              if route_of(r) == rk and r.get("scenario") == sc
                              and r.get("run_index") == i and r.get("counts")})
                cells.append(" / ".join(f"靜{a}動{b}" for a, b in got) if got else "—")
            lead = f"| {rk} | " if multi else "| "
            out.append(f"{lead}{SCEN_ZH[sc]} | " + " | ".join(cells) + " |")
    return out


def render(runs, has_browse_tree: bool = True) -> str:
    L = []
    A = L.append
    A("# 論文錄影產物")
    A("")
    A("Isaac Sim 裡跑**與實車同一套 ROS stack**（ndt_localizer + campusrover_routing")
    A("+ rover_rl policy + vo_safety_node）的 3F 走廊來回導航。")
    A("")
    models = sorted({r["model"] for r in runs})
    routes = sorted({route_of(r) for r in runs})
    scens = ["static", "dynamic", "mixed"]
    A(f"**{len(models)} 模型 × {len(routes)} 路線 × {len(scens)} 情境 × 4 難度 × 3 視角 = "
      f"{len(runs) * 3} 段影片**，每趟附一份命名同步的 rosbag。")
    A("")
    A("路線：" + "、".join(f"`{rk}`（c28 ↔ {rk}）" for rk in routes if rk)
      + "。主路線是 c27；c36 是 c27 再往西延伸 3.8 m。")
    A("")
    # 對照組的資料夾裡沒有索引樹，這段要是照印就會叫人去一個不存在的地方。
    if has_browse_tree:
        A("## 要挑影片看 → 找「中文影片」資料夾")
        A("")
        A("裡面是**中文命名的符號連結**，檔名直接寫清楚是哪個模型、什麼情境、")
        A("第幾趟、場上有幾個靜態物幾個動態物、哪個機位。")
        A("")
    else:
        A("## 檔名怎麼看")
        A("")
        A("這一組的影片在各趟的 `video/` 底下（`<tag>_{topdown,chase,oblique}.mp4`）。")
        A("中文命名的索引在主批次那邊：`recordings/00_影片總覽/`，")
        A("這一組在它的 `02`~`04` 區塊底下。")
        A("")
    if has_browse_tree:
        A("兩個地方都有，內容一樣（同一支程式產生，不會走樣）：")
        A("")
        A("```")
        A("recordings/模型sa4r2/00_中文影片/    ← 點進某個模型時就在眼前")
        A("recordings/00_影片總覽/              ← 三個模型 + 三組對照，一次看全")
        A("```")
        A("")
        A("```")
        A("00_影片總覽/")
        A("    01_主批次_三模型正式錄影/模型sa5r2/混合_ORCA互動_第4趟_靜6動8_俯視.mp4")
        A("    02_對照_行人走固定路線_非ORCA/   03_對照_速度0.6/   04_對照_速度1.0/")
        A("    05_每趟原始資料_bag與CSV/模型sa5r2_混合_ORCA互動_第4趟_靜6動8/")
        A("```")
        A("")
    if has_browse_tree:
        A("索引裡的影片是 **hard link**，在檔案總管裡就是普通檔案：**可以直接複製、")
        A("拖拉、上傳**，而且和本體共用同一份資料、不佔額外空間。")
        A("")
        A("⚠ 以前用的是捷徑（符號連結），複製到別的資料夾就打不開 —— 因為捷徑裡")
        A("寫的是相對路徑，換了層數就指到不存在的地方。已改掉。")
        A("")
        A("你可以在 `00_影片總覽/` 底下自己開資料夾（例如 `上傳/`）放要給人的片子，")
        A("重建索引**不會**刪掉它 —— 只有 `NN_` 開頭的區塊是程式管的。")
        A("")
        A("⚠ 因為是 hard link，`du -sh recordings` 會把對照組的影片也算進來")
        A("（本體在 `recordings_abl/`）。要看真實總量請一起算：")
        A("`du -csh recordings recordings_abl`（目前 74G）。")
        A("")
    A("索引裡的檔名格式：**`{情境}_{行人走法}_第N趟_靜{靜態數}動{動態數}_{機位}`**")
    A("")
    A("| 欄位 | 值 | 原文 | 意思 |")
    A("|---|---|---|---|")
    A("| 情境 | 純靜態 | `static` | 場上沒有任何會動的東西 |")
    A("| | 純動態 | `dynamic` | 障礙圓柱全關，但**站立人物仍在場**（見下） |")
    A("| | 混合 | `mixed` | 障礙 + 走動行人都有 |")
    A("| 行人走法 | ORCA互動 | `crowd_mode=orca` | RVO2，會閃避彼此**與車** |")
    A("| | 固定路線 | `crowd_mode=path` | 沿固定路線來回，不理車（只有對照1）|")
    A("| | 不走動 | — | 沒有人在走（純靜態）|")
    A("| 機位 | 俯視 / 車後 / 斜前方 | `topdown` / `chase` / `oblique` | 三個相機 |")
    A("")
    A("⚠ **`crowd_mode` 不能直接當標籤。** `static` 那幾趟的 `run.json` 照樣記")
    A("`crowd_mode: orca`（那只是 CLI 參數），但 `walks_enabled=False`、實際沒有人")
    A("在走。所以標籤看**實際動態數**：動態為 0 就標「不走動」。")
    A("")
    A("### 靜/動數量是怎麼數的")
    A("")
    A("一律從**該趟自己的 `isaac_nav.log`** 數（`scripts/scene_counts.py`），")
    A("不是查 `scene_variants` 的計畫表 —— 執行期會停用太靠近車、站在 routing")
    A("點上、或擠在一起的角色，拿計畫值寫進檔名等於讓檔名有機會說謊。")
    A("")
    A("```")
    A("靜態數 = 啟用的障礙圓柱/箱 + 不會動的角色 − 重疊（站立人物疊在 person 障礙上）")
    A("動態數 = 沿路徑移動的角色")
    A("```")
    A("")
    A("各情境 × 各趟實際在場的數量（**由各趟的 log 數出來**，不是抄計畫表）：")
    A("")
    for line in density_table(runs):
        A(line)
    A("")
    A("重建索引（重錄後跑一次就好，只建連結、不複製檔案）：")
    A("")
    A("```bash")
    A("python3 scripts/make_browse_tree.py")
    A("```")
    A("")
    A("⚠ **真資料夾名（`sa4r2_mixed_run04`）刻意不改成中文。** tag 被寫進")
    A("`run.json` 的 `tag` 欄、`nav/<tag>_leg?_*.csv`、`video/<tag>_*.mp4`，而")
    A("`record_batch.sh ONLY=<tag>` 與所有分析程式都靠它對應；改名會讓「重跑某一趟」")
    A("和全部分析失效，而且重跑批次又會把舊名字生回來。")
    A("")
    A("## 場景")
    A("")
    A("| 情境 | 靜態障礙（人形 / 道具） | 走動人物 |")
    A("|---|---|---|")
    A("| `static` | 啟用 | 不走動、停在 USD 原位（**也算靜態**）|")
    A("| `dynamic` | **全部關閉**（連站立人物也停用） | ORCA 互動避讓 |")
    A("| `mixed` | 啟用 | ORCA 互動避讓 |")
    A("")
    A("靜態障礙每個都**隨機抽**成「站立行人」或「Isaac 官方道具」（置物櫃、貨架、")
    A("盆栽、檔案櫃），抽法見 `scripts/scene_variants.py` 的 `_place_obstacles`。")
    A("道具一律配一個**隱形 bbox 碰撞盒** —— 光達是 PhysX 光達、只打碰撞體，")
    A("道具只放模型的話光達看不到。可用道具清單與高度實測見 `scripts/props.py`。")
    A("")
    A("### 舊批次（c28↔c25）的 `dynamic` 其實有靜態障礙 —— 已修正")
    A("")
    A("舊版 `place_standing` 不看 `obstacles_enabled`，`dynamic` 場上仍有 3~5 個")
    A("站立人物，與 `mixed` 的靜態內容幾乎相同：")
    A("")
    A("| 難度 | `dynamic` | `mixed` | 差別 |")
    A("|---|---|---|---|")
    A("| 第1趟 | 靜3動2 | 靜3動2 | **完全相同** |")
    A("| 第2趟 | 靜4動4 | 靜4動4 | **完全相同** |")
    A("| 第3趟 | 靜4動6 | 靜5動6 | 差 1 個（箱型障礙）|")
    A("| 第4趟 | 靜5動8 | 靜6動8 | 差 1 個（箱型障礙）|")
    A("")
    A("**上表是舊路線（c28↔c25）那一批。已修正：** 障礙關閉時站立人物整個停用，")
    A("新批次的 `dynamic` 場上真的沒有靜態物。舊批次的 `dynamic` 不可當純動態引用。")
    A("")
    A("⚠ 更正：這裡先前寫「velodyne 是 RTX 光達、看不到隱形圓柱」——**錯**。")
    A("發佈 `/velodyne_points_ideal` 的是 `isaacsim.sensors.physx.IsaacReadLidarPointCloud`，")
    A("讀的是 PhysX `Lidar` prim（stage 裡另有一個 `RTX_Lidar`，但沒有接到發佈節點）。")
    A("PhysX 光達**只打物理碰撞體**，隱形圓柱看得到、只有算圖網格沒碰撞體的東西才看不到。")
    A("")
    A("### `static` 的走動人物沒有消失")
    A("")
    A("`static` 的走動人物不是消失，而是**停在 USD 原始位置**；其中")
    A("Character_10~13、19 的原位剛好就在**走廊中線上** —— 這就是 static")
    A("那幾趟耗時最長、最近距離最差的原因，也是它靜態數最高（最多 14）的原因。")
    A("")
    A("每趟（run01→run04）的障礙與行人**數量遞增、位置各不相同**：")
    A("3/4/5/6 個障礙、2/4/6/8 個走動行人。走廊中段固定有**兩人肩並肩站在一側**，")
    A("逼車走另一邊；其餘障礙沿走廊左右交錯。人形障礙是真的虛擬人物")
    A("（不可見的圓柱負責物理碰撞，人物負責外觀與光達輪廓）。")
    A("")
    A("## 結果")
    A("")
    A("碰撞幀 = LiDAR 最近距離 ≤0.45 m 的取樣點數（車體半徑 0.35 + 緩衝 0.10），")
    A("**不是**物理碰撞事件。")
    A("")
    A("### 「最近障礙」到底是靠近了什麼")
    A("")
    A("碰撞幀的數字若有一部分其實是**牆**，這個指標就不能當論文的避障證據 ——")
    A("走廊很窄，貼著牆走是正常的。所以逐幀把最近距離拆成四個來源")
    A("（`scripts/nearest_source.py`：車的真值位姿 + 行人軌跡 + 該趟障礙表 +")
    A("佔據圖距離場，對齊後與 nav CSV 實測的 `min_range_m` 逐點核對）。")
    A("")
    A("36 趟 / 33092 個取樣點，「最近的東西」是誰：")
    A("")
    A("| 來源 | 佔比 |")
    A("|---|---|")
    A("| 站立行人（含人形障礙）| 44.1% |")
    A("| 牆 | 29.1% |")
    A("| 走動行人 | 22.5% |")
    A("| 箱型障礙 | 4.3% |")
    A("")
    A("**11 個碰撞幀（實測 ≤0.45 m）的來源：站立行人 6、箱型障礙 1、來源不明 4 ——")
    A("沒有一個是牆。** 碰撞幀的數字沒有被牆污染。")
    A("")
    A("⚠ 那 4 個「來源不明」全在 `sa4r3_mixed_run04`：實測 0.447 m，但場上最近的")
    A("已知物體在 1.75 m 外。把那片回波用 bag 的 TF 搬到 map 之後（同一片雲有")
    A("65% 的點落在佔據圖的牆上 0.05 m 內，所以 TF 沒問題），它落在離最近牆")
    A("2.25 m 的空地、車正後方 0.45 m，**隨車一起移動**；佔據圖、建物 Mesh、")
    A("18 個障礙 prim、13 個角色的位置全都對不上。")
    A("")
    A("它**不可能是光達的原始回波**：PhysX 光達 `minRange = 0.5 m`，可見帶仰角 ±15°，")
    A("真實回波的水平距離最小是 0.5 × cos15° = **0.483 m**，而這些點是 0.447 m。")
    A("`/velodyne_points` 是後處理節點（運動模糊）從 `/velodyne_points_ideal` 產生的，")
    A("這些點是被它往內搬過的。（先前說成「被自己的 RTX 光達打到」—— 機制講錯。）")
    A("舊批次 bag 沒錄 `_ideal`，無法直接比對；新批次已加錄。")
    A("**引用碰撞幀時，`sa4r3` mixed 那 4 幀應當排除（該列實為 0）。**")
    A("")
    A("### ⚠⚠ 碰撞幀這個指標本身的限制")
    A("")
    A("光達量得到的最近距離在 **約 0.45~0.5 m 就飽和**（minRange 0.5 m），")
    A("比這更近的真實距離它量不到。所以「光達最近距離 ≤0.45 m」這個碰撞幀定義")
    A("兩個方向都不可靠：")
    A("")
    A("- 真的很近時未必觸發 —— 讀值卡在 0.48 附近，要靠後處理剛好往內搬才過門檻")
    A("- 沒有東西時也可能觸發 —— 上面那 4 個來源不明就是")
    A("")
    A("上表 7 個有來源的碰撞幀，真值幾何算出的表面距離是 **0.26~0.39 m**，")
    A("光達卻讀成 0.42~0.45 m。**論文要講「多近」請用真值幾何的距離**")
    A("（`scripts/nearest_source.py` 的預測值），不要用光達讀值。")
    A("")
    A("車體半徑約 0.35 m，真實距離 0.26 m 代表車身**可能已經碰到**障礙物。")
    A("新批次起每一趟都記「真的碰到」（逐趟明細表的欄位，`collisions.csv`）：")
    A("每一步拿外觀車身大小的盒子問物理引擎有沒有別人的碰撞體，不受光達限制。")
    A("物理引擎裡的底盤碰撞體只是一片 0.17 × 0.47 m 的薄板，行人又對車做了")
    A("接觸過濾，所以**不能用物理碰撞回報**——兩者都會讓擦撞顯示成 0；實測它還會")
    A("把「底盤薄板插在地板裡」記成整趟撞牆，已拿掉。")
    A("")
    A("方法本身的誤差：預測−實測的逐趟中位都落在 −0.22 ~ 0.00 m。p05 到 −0.65 m ——")
    A("那是佔據圖上有實機掃描留下的雜物、模擬場景裡沒有，所以「離牆距離」偏小。")
    A("這只會**高估**牆的佔比，不影響「沒有一個碰撞幀是牆」的結論。")
    A("")
    A("重算：`PYTHONPATH= .venv/bin/python scripts/nearest_source.py recordings`")
    A("（完整報表在 `reports/nearest_source.txt`）")
    A("")
    A("### 依模型 × 路線 × 情境彙總")
    A("")
    A("| 模型 | 路線 | 情境 | 抵達 | 最近障礙(最差) | 碰撞幀 | 真的碰到 | 平均每段耗時 |")
    A("|---|---|---|---|---|---|---|---|")
    for m in models:
        for rk in routes:
            for sc in scens:
                sel = [r for r in runs if r["model"] == m and route_of(r) == rk
                       and r["scenario"] == sc]
                legs = [l for r in sel for l in r["legs"]]
                c = cell_summary(legs)
                touched = sum((r.get("collisions") or {}).get("episodes", 0) for r in sel)
                broken = any((r.get("collisions") or {}).get("detector", {}).get("overlap_ok") is False
                             for r in sel)
                touch_cell = "⚠ 偵測器失效" if broken else str(touched)
                if not c["legs"]:
                    A(f"| `{m}` | {rk} | {sc} | — | — | — | — | — |")
                    continue
                A(f"| `{m}` | {rk} | {sc} | {c['arrived']}/{c['legs']} 段 | "
                  f"{c['min_range_m']:.2f} m | {c['collision_frames']} | {touch_cell} | "
                  f"{c['seconds']:.1f} s |")
    A("")
    A("### 逐趟明細")
    A("")
    A("| tag | 模型 | 情境 | 難度 | 抵達 | 最近障礙 | 碰撞幀 | 真的碰到 | 影片 |")
    A("|---|---|---|---|---|---|---|---|---|")
    for r in sorted(runs, key=lambda r: r["tag"]):
        c = cell_summary(r["legs"])
        mr = f"{c['min_range_m']:.2f} m" if c["min_range_m"] is not None else "—"
        A(f"| `{r['tag']}` | {r['model']} | {r['scenario']} | run{r['run_index']:02d} | "
          f"{c['arrived']}/{c['legs']} | {mr} | {c['collision_frames']} | "
          f"{collision_cell(r.get('collisions'))} | {len(r.get('videos', []))} |")
    A("")
    A("「真的碰到」= 外觀車身盒子與別人的碰撞體重疊（`scripts/collision_log.py`），")
    A("不受光達 0.5 m 最小量測距離限制。**「偵測器失效」不是零次**，是那一趟")
    A("偵測器沒過檢查（重疊查詢沒看到車自己、或開跑後故意去查一個障礙／行人卻查不到），")
    A("那一趟的擦撞次數不可信。")
    A("")
    A("## 目錄長相")
    A("")
    A("```")
    A("recordings/")
    A("    00_影片總覽/                            ← 先看這裡：中文命名的影片")
    A("        01_正式錄影_依模型_路線_情境分類/")
    A("            模型sa4r2/  模型sa4r3/  模型sa5r2/")
    A("                路線A_c28往返c27_主路線/")
    A("                路線B_c28往返c36_延伸到c36/")
    A("                    1_純靜態_只有靜止障礙/")
    A("                    2_動態_只有走動行人/")
    A("                    3_混合_靜止障礙加走動行人/")
    A("                        sa4r2_路線c27_混合_ORCA互動_第4趟_靜6動8_俯視.mp4")
    A("        05_每趟原始資料_bag與CSV/           ← 每趟一個捷徑，指到下面的原始資料夾")
    A("        上傳/                               ← 使用者自己的資料夾，程式不會動")
    A("    模型sa4r2/  模型sa4r3/  模型sa5r2/     ← 原始資料（程式用，資料夾名不要改）")
    A("        00_中文影片/                        該模型 72 段，同樣分 路線/情境 兩層")
    A("        <模型>_<路線>_<情境>_run<NN>/       各 24 趟的原始資料，內容見下")
    A("    _舊版與作廢錄影_保留不刪/               舊路線、舊擺法、有 bug 的趟；**勿引用**，")
    A("                                            各資料夾的說明見裡面的 README.md")
    A("    batch.log  README.md")
    A("")
    A("recordings_abl/")
    A("    _舊版錄影_保留不刪/                     舊路線 c28↔c25 的三組對照；新路線未重錄")
    A("```")
    A("")
    A("⚠ 「什麼算一趟」的唯一定義在 `scripts/run_layout.py` 的 `run_dirs()` ——")
    A("它會同時看 root 底下與模型子資料夾底下兩層，並排除索引樹裡的符號連結")
    A("（不排除的話每趟會被算兩次）。**新增分析程式請用它，不要自己列目錄。**")
    A("")
    A("```")
    A("recordings/模型<模型>/<模型>_<情境>_run<NN>/")
    A("    video/<tag>_{topdown,chase,oblique}.mp4")
    A("    bag/<tag>/                 ros2 bag（mcap），含 RViz 需要的全部 topic")
    A("    nav/<tag>_leg{1,2}_*.csv   逐時刻：位置 / VO / 最近障礙 / 命令速度")
    A("    pose.csv / crowd.csv       車與行人的世界位姿（第二遍回放用）")
    A("    frame_times.csv            幀號 ↔ 模擬時間")
    A("    run.json                   參數、導航結果、影片↔bag 對齊偏移")
    A("```")
    A("")
    A("## 影片與 rosbag 怎麼對齊")
    A("")
    A("```bash")
    A("ros2 bag play recordings/<tag>/bag/<tag> --clock \\")
    A("    --start-offset $(jq -r .sync.bag_play_start_offset_s recordings/<tag>/run.json)")
    A("```")
    A("影片第 N 幀 = 上面的 offset 再加 N/30 秒。")
    A("")
    A("## 怎麼重做")
    A("")
    A("```bash")
    A("bash scripts/record_batch.sh                    # 整批（已完成的自動跳過）")
    A("ONLY=sa4r2_mixed_run02 bash scripts/record_batch.sh   # 只重跑某一趟")
    A("python3 scripts/make_browse_tree.py                   # 重建中文索引")
    A("python3 scripts/make_readme.py recordings > recordings/README.md")
    A("```")
    A("")
    A("⚠ 這份 README 是 `make_readme.py` 產生的 —— **不要手改**，改了下次重新產生")
    A("就會被蓋掉。要改內容請改 `scripts/make_readme.py` 的 `render()`。")
    A("")
    A("⚠ 一趟要跑**兩遍**（見 `scripts/pose_log.py` 檔頭）：path tracing 會把")
    A("RTF 壓到 0.35，邊導航邊算圖時 cmd_vel 被釘在 0.060 m/s（正常 0.475），")
    A("到不了終點。所以第一遍全速導航＋錄 bag＋寫位姿，第二遍照位姿回放算圖。")
    return "\n".join(L) + "\n"


def main(root: Path) -> int:
    runs = []
    for d in run_dirs(root):
        mp = d / "run.json"
        if not mp.exists():
            continue
        meta = json.loads(mp.read_text())
        nav = (d / "nav.log").read_text(errors="replace") if (d / "nav.log").exists() else ""
        meta["legs"] = parse_nav_table(nav)
        log = d / "isaac_nav.log"
        meta["counts"] = (counts_from_log(log.read_text(errors="replace"))
                          if log.exists() else None)
        runs.append(meta)
    if not runs:
        print("（找不到任何 run.json）", file=sys.stderr)
        return 1
    print(render(runs, (root / BROWSE_DIRNAME).is_dir()), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(Path(sys.argv[1] if len(sys.argv) > 1 else "recordings")))
