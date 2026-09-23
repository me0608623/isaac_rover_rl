#!/usr/bin/env python3
"""從各趟的 run.json / nav.log 產生結果 README。

用法：  python3 scripts/make_readme.py recordings > recordings/README.md
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

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


def render(runs) -> str:
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
    A("## 場景")
    A("")
    A("| 情境 | 靜態障礙 | 行人 |")
    A("|---|---|---|")
    A("| `static` | 有 | 站著（不走動）|")
    A("| `dynamic` | 無 | ORCA 互動避讓 |")
    A("| `mixed` | 有 | ORCA 互動避讓 |")
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
    A("recordings/<模型>_<情境>_run<NN>/")
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
    A("```")
    A("")
    A("⚠ 一趟要跑**兩遍**（見 `scripts/pose_log.py` 檔頭）：path tracing 會把")
    A("RTF 壓到 0.35，邊導航邊算圖時 cmd_vel 被釘在 0.060 m/s（正常 0.475），")
    A("到不了終點。所以第一遍全速導航＋錄 bag＋寫位姿，第二遍照位姿回放算圖。")
    return "\n".join(L) + "\n"


def main(root: Path) -> int:
    runs = []
    for d in sorted(p for p in root.iterdir() if p.is_dir() and not p.name.startswith("_")):
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
    print(render(runs), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(Path(sys.argv[1] if len(sys.argv) > 1 else "recordings")))
