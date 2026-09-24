#!/usr/bin/env python3
"""重算每一段的**實際行駛時間**（2026-09-24）。

⚠ 原本以為是「routing 偶爾要 ~55 s 才給路徑」—— **錯**。查證後：routing 6 ms 就回、
車 1 s 內開動。真正原因是 monitor_navigation 用模擬時間計時，節點剛建好還沒收到
/clock 時 now() = 0，第一段的 t0 被記成 0，「耗時」多算了開跑前的整段模擬時間
（~55~62 s）。72 趟裡 27 趟的去程中招，已在 monitor_navigation 修正（2c8f33a）。

逐時刻記錄 nav/<tag>_leg?_*.csv 的第一筆是車開始收到 policy 指令的時刻、最後一筆
是抵達那一刻，所以：

    實際行駛時間 = csv 最後一筆的 t（t 已從第一筆歸零，模擬秒）
    計時誤差     = 原耗時 − 實際行駛時間

用法：python3 scripts/driving_time.py recordings > reports/driving_time.md
"""重算每一段的**實際行駛時間**，扣掉等 routing 給路徑的時間（2026-09-24）。

為什麼要重算：monitor_navigation 的「耗時」從送出 routing 請求起算，但
routing_to_path 偶爾要 ~55 s 才發 /global_path，車停在原地等 —— 三個模型都
遇到過，跟模型無關，卻把去程耗時灌水到 80~110 s。

逐時刻記錄 nav/<tag>_leg?_*.csv 的第一筆是車開始收到 policy 指令的時刻
（policy 收到路徑才會輸出），最後一筆是抵達那一刻，所以：

    實際行駛時間 = csv 最後一筆的 t（t 已從第一筆歸零，模擬秒）
    等待路徑時間 = 原耗時 − 實際行駛時間

用法：python3 scripts/driving_time.py recordings > reports/driving_time.md
"""重算每一段的**實際行駛時間**（2026-09-24）。

⚠ 原本以為是「routing 偶爾要 ~55 s 才給路徑」—— **錯**。查證後：routing 6 ms 就回、
車 1 s 內開動。真正原因是 monitor_navigation 用模擬時間計時，節點剛建好還沒收到
/clock 時 now() = 0，第一段的 t0 被記成 0，「耗時」多算了開跑前的整段模擬時間
（~55~62 s）。72 趟裡 27 趟的去程中招，已在 monitor_navigation 修正（2c8f33a）。

逐時刻記錄 nav/<tag>_leg?_*.csv 的第一筆是車開始收到 policy 指令的時刻、最後一筆
是抵達那一刻，所以：

    實際行駛時間 = csv 最後一筆的 t（t 已從第一筆歸零，模擬秒）
    計時誤差     = 原耗時 − 實際行駛時間

用法：python3 scripts/driving_time.py recordings > reports/driving_time.md
"""

from __future__ import annotations

import csv
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from browse_names import SCENARIO_ZH
from run_layout import run_dirs

_ROW = re.compile(r"^(c\d+)→(c\d+)\s+(OK|FAIL)\s+([\d.]+)s\s+([\d.]+)m", re.M)


def legs_of(run_dir: Path):
    """重算每一段的**實際行駛時間**（2026-09-24）。

⚠ 原本以為是「routing 偶爾要 ~55 s 才給路徑」—— **錯**。查證後：routing 6 ms 就回、
車 1 s 內開動。真正原因是 monitor_navigation 用模擬時間計時，節點剛建好還沒收到
/clock 時 now() = 0，第一段的 t0 被記成 0，「耗時」多算了開跑前的整段模擬時間
（~55~62 s）。72 趟裡 27 趟的去程中招，已在 monitor_navigation 修正（2c8f33a）。

逐時刻記錄 nav/<tag>_leg?_*.csv 的第一筆是車開始收到 policy 指令的時刻、最後一筆
是抵達那一刻，所以：

    實際行駛時間 = csv 最後一筆的 t（t 已從第一筆歸零，模擬秒）
    計時誤差     = 原耗時 − 實際行駛時間

用法：python3 scripts/driving_time.py recordings > reports/driving_time.md
"""回傳 ``[(段名, ok, 原耗時, 路徑m, 行駛秒)]``。"""重算每一段的**實際行駛時間**（2026-09-24）。

⚠ 原本以為是「routing 偶爾要 ~55 s 才給路徑」—— **錯**。查證後：routing 6 ms 就回、
車 1 s 內開動。真正原因是 monitor_navigation 用模擬時間計時，節點剛建好還沒收到
/clock 時 now() = 0，第一段的 t0 被記成 0，「耗時」多算了開跑前的整段模擬時間
（~55~62 s）。72 趟裡 27 趟的去程中招，已在 monitor_navigation 修正（2c8f33a）。

逐時刻記錄 nav/<tag>_leg?_*.csv 的第一筆是車開始收到 policy 指令的時刻、最後一筆
是抵達那一刻，所以：

    實際行駛時間 = csv 最後一筆的 t（t 已從第一筆歸零，模擬秒）
    計時誤差     = 原耗時 − 實際行駛時間

用法：python3 scripts/driving_time.py recordings > reports/driving_time.md
"""
    nav = (run_dir / "nav.log").read_text(errors="replace")
    rows = _ROW.findall(nav)
    out = []
    for i, (a, b, ok, t, dist) in enumerate(rows, 1):
        f = next(iter(sorted((run_dir / "nav").glob(f"*leg{i}_{a}_to_{b}.csv"))), None)
        drive = None
        if f is not None:
            ts = [float(r["t"]) for r in csv.DictReader(open(f))]
            drive = ts[-1] if ts else None
        out.append((f"{a}→{b}", ok == "OK", float(t), float(dist), drive))
    return out


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "recordings")
    per_run = []
    for d in run_dirs(root):
        meta = json.loads((d / "run.json").read_text())
        model = meta.get("model") or meta["tag"].split("_", 1)[0]
        route = meta.get("route_key") or ""
        for k, (leg, ok, t, dist, drive) in enumerate(legs_of(d)):
            per_run.append(dict(model=model, route=route, scenario=meta["scenario"],
                                run=int(meta["run_index"]), leg="去" if k == 0 else "回",
                                seg=leg, ok=ok, total=t, dist=dist, drive=drive,
                                wait=(t - drive) if drive is not None else None))
    out_csv = Path("reports/driving_time.csv")
    out_csv.parent.mkdir(exist_ok=True)
    with open(out_csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(per_run[0]))
        w.writeheader()
        w.writerows(per_run)

    P = print
    P("# 實際行駛時間（修正計時錯誤）\n")
    P(f"共 {len(per_run)} 段（{len(per_run)//2} 趟 × 去回）。逐段明細：`{out_csv}`。\n")
    waits = [r["wait"] for r in per_run if r["wait"] is not None]
    long_w = [r for r in per_run if r["wait"] is not None and r["wait"] > 10]
    P(f"- 原耗時多算超過 10 s 的段：**{len(long_w)} / {len(per_run)}**，"
      f"全部是{'去程' if all(r['leg']=='去' for r in long_w) else '去程與回程'}；"
      f"平均多算 {sum(r['wait'] for r in long_w)/max(1,len(long_w)):.1f} s。"
      f"原因是監控程式第一段計時起點被讀成 0（還沒收到模擬時鐘），車並沒有停著等；已修正")
    P(f"- 其餘段差距中位 {sorted(waits)[len(waits)//2]:.1f} s\n")

    groups = defaultdict(list)
    for r in per_run:
        groups[(r["model"], r["route"], r["scenario"], r["leg"])].append(r)
    for route in sorted({r["route"] for r in per_run}):
        P(f"## 路線 {route}\n")
        P("| 模型 | 情境 | 去：行駛 s | 去：原耗時 s | 回：行駛 s | 回：原耗時 s | 平均路徑 m |")
        P("|---|---|---:|---:|---:|---:|---:|")
        for model in sorted({r["model"] for r in per_run}):
            for scen in ("static", "dynamic", "mixed"):
                go = groups.get((model, route, scen, "去"), [])
                bk = groups.get((model, route, scen, "回"), [])
                if not go:
                    continue
                mean = lambda xs: sum(xs) / len(xs) if xs else float("nan")
                P(f"| {model} | {SCENARIO_ZH[scen]} | {mean([r['drive'] for r in go]):.1f} | "
                  f"{mean([r['total'] for r in go]):.1f} | {mean([r['drive'] for r in bk]):.1f} | "
                  f"{mean([r['total'] for r in bk]):.1f} | "
                  f"{mean([r['dist'] for r in go + bk]):.1f} |")
        P("")
    P("每格是 4 趟的平均。「原耗時」有 27 段去程被多算，**論文請用「行駛」欄**。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
