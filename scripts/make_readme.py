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


def render(runs, has_browse_tree: bool = True) -> str:
    L = []
    A = L.append
    A("# 論文錄影產物")
    A("")
    A("Isaac Sim 裡跑**與實車同一套 ROS stack**（ndt_localizer + campusrover_routing")
    A("+ rover_rl policy + vo_safety_node）的 3F 走廊來回導航。")
    A("")
    models = sorted({r["model"] for r in runs})
    scens = ["static", "dynamic", "mixed"]
    A(f"**{len(models)} 模型 × {len(scens)} 情境 × 4 難度 × 3 視角 = "
      f"{len(runs) * 3} 段影片**，每趟附一份命名同步的 rosbag。")
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
    A("所以同一個「第4趟」在三個情境的實際密度差很多：")
    A("")
    A("| 情境 | 第1趟 | 第2趟 | 第3趟 | 第4趟 |")
    A("|---|---|---|---|---|")
    A("| 純靜態 | 靜5動0 | 靜8動0 | 靜11動0 | **靜14動0** |")
    A("| 純動態 | 靜3動2 | 靜4動4 | 靜4動6 | 靜5動8 |")
    A("| 混合 | 靜3動2 | 靜4動4 | 靜5動6 | 靜6動8 |")
    A("")
    A("**純靜態才是靜態物最密的情境**（走動人物停在原位也算靜態，而且")
    A("Character_10~13、19 的原位就在走廊中線上）—— 這就是 static 那幾趟")
    A("耗時最長、最近距離最差的原因。")
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
    A("| 情境 | 障礙圓柱/箱 | 站立人物 | 走動人物 |")
    A("|---|---|---|---|")
    A("| `static` | 啟用 | 在障礙位置 | **站在 USD 原位、不走動** |")
    A("| `dynamic` | 全部關閉 | **仍在場** | ORCA 互動避讓 |")
    A("| `mixed` | 啟用 | 在障礙位置 | ORCA 互動避讓 |")
    A("")
    A("⚠ 兩件容易誤會、2026-09-23 逐幀查證過的事（寫論文敘述時要照這個）：")
    A("")
    A("- `dynamic` 的 log 印「靜態障礙 0/18 啟用」**只關掉圓柱**，")
    A("  `place_standing` 擺的站立人物還在場上，光達照樣打得到。")
    A("- `static` 的走動人物不是消失，而是**停在 USD 原始位置**；")
    A("  其中 Character_10~13、19 的原位剛好就在**走廊中線上**——")
    A("  這就是 static 那幾趟耗時最長、最近距離最差的原因。")
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
    A("18 個障礙 prim、13 個角色的位置全都對不上。最可能是車體自身結構被自己的")
    A("RTX 光達打到（RTX 光達打的是算圖網格，不是物理碰撞體）。")
    A("**引用碰撞幀時，`sa4r3` mixed 那 4 幀應當排除（該列實為 0）。**")
    A("")
    A("方法本身的誤差：預測−實測的逐趟中位都落在 −0.22 ~ 0.00 m。p05 到 −0.65 m ——")
    A("那是佔據圖上有實機掃描留下的雜物、模擬場景裡沒有，所以「離牆距離」偏小。")
    A("這只會**高估**牆的佔比，不影響「沒有一個碰撞幀是牆」的結論。")
    A("")
    A("重算：`PYTHONPATH= .venv/bin/python scripts/nearest_source.py recordings`")
    A("（完整報表在 `reports/nearest_source.txt`）")
    A("")
    A("### 依模型 × 情境彙總")
    A("")
    A("| 模型 | 情境 | 抵達 | 最近障礙(最差) | 碰撞幀 | 平均每段耗時 |")
    A("|---|---|---|---|---|---|")
    for m in models:
        for sc in scens:
            legs = [l for r in runs if r["model"] == m and r["scenario"] == sc
                    for l in r["legs"]]
            c = cell_summary(legs)
            if not c["legs"]:
                A(f"| `{m}` | {sc} | — | — | — | — |")
                continue
            A(f"| `{m}` | {sc} | {c['arrived']}/{c['legs']} 段 | "
              f"{c['min_range_m']:.2f} m | {c['collision_frames']} | "
              f"{c['seconds']:.1f} s |")
    A("")
    A("### 逐趟明細")
    A("")
    A("| tag | 模型 | 情境 | 難度 | 抵達 | 最近障礙 | 碰撞幀 | 影片 |")
    A("|---|---|---|---|---|---|---|---|")
    for r in sorted(runs, key=lambda r: r["tag"]):
        c = cell_summary(r["legs"])
        mr = f"{c['min_range_m']:.2f} m" if c["min_range_m"] is not None else "—"
        A(f"| `{r['tag']}` | {r['model']} | {r['scenario']} | run{r['run_index']:02d} | "
          f"{c['arrived']}/{c['legs']} | {mr} | {c['collision_frames']} | "
          f"{len(r.get('videos', []))} |")
    A("")
    A("## 目錄長相")
    A("")
    A("```")
    A("recordings/")
    A("    00_影片總覽/               中文命名的符號連結索引（見上面）")
    A("    模型sa4r2/  模型sa4r3/  模型sa5r2/      ← 先按模型分類")
    A("        00_中文影片/           該模型 36 段的中文連結")
    A("        <模型>_<情境>_run<NN>/ 各 12 趟的原始資料，內容見下")
    A("    _作廢批次_只留紀錄/       兩份被取代的舊批次，影片與 bag 已刪（省 20 G），")
    A("                              只留 log 與 run.json；數字**勿引用**：")
    A("                              v1 輪徑未修正、車速偏快 11%；v2 行人還走直線未用 ORCA")
    A("    batch.log  README.md")
    A("")
    A("recordings_abl/               三組對照（各 12 趟，單一模型 sa4r2 故無模型層）")
    A("    對照1_行人走固定路線_非ORCA/   ← 唯一不用 ORCA 的一組")
    A("    對照2_速度0.6/                 ← 仍是 ORCA，只改車速上限")
    A("    對照3_速度1.0/                 ← 仍是 ORCA，只改車速上限")
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
        runs.append(meta)
    if not runs:
        print("（找不到任何 run.json）", file=sys.stderr)
        return 1
    print(render(runs, (root / BROWSE_DIRNAME).is_dir()), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(Path(sys.argv[1] if len(sys.argv) > 1 else "recordings")))
