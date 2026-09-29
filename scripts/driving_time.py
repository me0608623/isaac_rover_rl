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
"""

from __future__ import annotations

import csv
import argparse
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from browse_names import SCENARIO_ZH
from run_layout import run_dirs

_ROW = re.compile(r"^(c\d+)→(c\d+)\s+(OK|FAIL)\s+([\d.]+)s\s+([\d.]+)m", re.M)

STUCK_MIN_SECONDS = 5.0
STUCK_RADIUS_M = 0.10


def nav_samples(path: Path) -> list[tuple[float, float, float]]:
    """讀逐時刻導航 CSV 的 ``(t, map_x, map_y)``；壞列直接跳過。"""
    out = []
    if path is None or not path.exists():
        return out
    with path.open(newline="") as fh:
        for row in csv.DictReader(fh):
            try:
                values = (float(row["t"]), float(row["map_x"]), float(row["map_y"]))
            except (KeyError, TypeError, ValueError):
                continue
            if all(math.isfinite(v) for v in values):
                out.append(values)
    return out


def stuck_intervals(samples, min_seconds: float = STUCK_MIN_SECONDS,
                    radius_m: float = STUCK_RADIUS_M) -> list[tuple[float, float]]:
    """找出車連續至少 ``min_seconds`` 都沒離開半徑 ``radius_m`` 的區段。

    位置取逐段記錄的 map 軌跡；這是「卡住」的可重算定義，不拿 ``cmd_v=0``
    當真值（安全煞或 policy 可以有命令，但車實際沒動）。
    """
    if len(samples) < 2:
        return []
    out = []
    start = 0
    for i in range(1, len(samples)):
        _t0, x0, y0 = samples[start]
        t, x, y = samples[i]
        if math.hypot(x - x0, y - y0) <= radius_m:
            continue
        if samples[i - 1][0] - samples[start][0] >= min_seconds:
            out.append((samples[start][0], samples[i - 1][0]))
        start = i
    if samples[-1][0] - samples[start][0] >= min_seconds:
        out.append((samples[start][0], samples[-1][0]))
    return out


def legs_of(run_dir: Path):
    """回傳逐段結果；新增欄位不影響舊錄影，缺資料明確回 ``None``。"""
    nav = (run_dir / "nav.log").read_text(errors="replace")
    rows = _ROW.findall(nav)
    out = []
    for i, (a, b, ok, t, dist) in enumerate(rows, 1):
        f = next(iter(sorted((run_dir / "nav").glob(f"*leg{i}_{a}_to_{b}.csv"))), None)
        drive = None
        samples = nav_samples(f)
        if samples:
            drive = samples[-1][0]
        stuck = stuck_intervals(samples)
        out.append({"seg": f"{a}→{b}", "ok": ok == "OK", "total": float(t),
                    "dist": float(dist), "drive": drive,
                    "stuck_segments": len(stuck),
                    "stuck_seconds": sum(b - a for a, b in stuck)})
    return out


def collect(root: Path):
    per_run = []
    for d in run_dirs(root):
        meta = json.loads((d / "run.json").read_text())
        model = meta.get("model") or meta["tag"].split("_", 1)[0]
        route = meta.get("route_key") or ""
        timeout_limit = meta.get("leg_timeout_s")
        clearance = ((meta.get("collisions") or {}).get("clearance") or {})
        clearance_m = clearance.get("min_m") if clearance.get("complete") else None
        for k, leg in enumerate(legs_of(d)):
            drive = leg["drive"]
            if leg["ok"]:
                timeout = False
            elif timeout_limit is None:
                timeout = None
            else:
                timeout = leg["total"] >= float(timeout_limit) - 1.0
            per_run.append(dict(
                tag=meta.get("tag", d.name), model=model, route=route,
                density=meta.get("density") or "",
                scenario=meta["scenario"], run=int(meta["run_index"]),
                leg="去" if k == 0 else "回", seg=leg["seg"], ok=leg["ok"],
                timeout=timeout, total=leg["total"], dist=leg["dist"], drive=drive,
                wait=(leg["total"] - drive) if drive is not None else None,
                stuck_segments=leg["stuck_segments"],
                stuck_seconds=round(leg["stuck_seconds"], 3),
                truth_clearance_m=clearance_m,
                truth_clearance_lower_bound_m=(clearance.get("lower_bound_m")
                                                if clearance.get("complete") else None),
                truth_clearance_method=clearance.get("method") or ""))
    return per_run


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("root", nargs="?", default="recordings")
    ap.add_argument("--csv", type=Path, default=None,
                    help="逐段 CSV 輸出；壓力資料預設寫在 root 裡，避免覆蓋正式報告")
    args = ap.parse_args(argv)
    root = Path(args.root)
    per_run = collect(root)
    if not per_run:
        print("（沒有可分析的導航段）", file=sys.stderr)
        return 1
    out_csv = args.csv or (Path("reports/driving_time.csv")
                           if root.name == "recordings" else root / "driving_time.csv")
    out_csv.parent.mkdir(exist_ok=True)
    stress = any(r["density"] for r in per_run)
    legacy_fields = ["model", "route", "scenario", "run", "leg", "seg", "ok",
                     "total", "dist", "drive", "wait"]
    fields = list(per_run[0]) if stress else legacy_fields
    with open(out_csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(per_run)

    P = print
    P("# 實際行駛時間（修正計時錯誤）\n")
    P((f"共 {len(per_run)} 段。" if stress else
       f"共 {len(per_run)} 段（{len(per_run)//2} 趟 × 去回）。")
      + f"逐段明細：`{out_csv}`。\n")
    waits = [r["wait"] for r in per_run if r["wait"] is not None]
    long_w = [r for r in per_run if r["wait"] is not None and r["wait"] > 10]
    P(f"- 原耗時多算超過 10 s 的段：**{len(long_w)} / {len(per_run)}**，"
      f"全部是{'去程' if all(r['leg']=='去' for r in long_w) else '去程與回程'}；"
      f"平均多算 {sum(r['wait'] for r in long_w)/max(1,len(long_w)):.1f} s；"
      f"原因是監控程式第一段計時起點被讀成 0（還沒收到模擬時鐘），車並沒有停著等，已修正")
    if waits:
        P(f"- 其餘段差距中位 {sorted(waits)[len(waits)//2]:.1f} s\n")
    else:
        P("- 沒有可計算的行駛時間 CSV。\n")

    if not stress:
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
                    P(f"| {model} | {SCENARIO_ZH[scen]} | "
                      f"{mean([r['drive'] for r in go]):.1f} | "
                      f"{mean([r['total'] for r in go]):.1f} | "
                      f"{mean([r['drive'] for r in bk]):.1f} | "
                      f"{mean([r['total'] for r in bk]):.1f} | "
                      f"{mean([r['dist'] for r in go + bk]):.1f} |")
            P("")
        P("每格是 4 趟的平均。「原耗時」有 27 段去程被多算，**論文請用「行駛」欄**。")
        return 0

    P(f"- 卡住定義：連續 ≥{STUCK_MIN_SECONDS:.0f} s 沒離開半徑 "
      f"{STUCK_RADIUS_M:.2f} m；合計 **{sum(r['stuck_segments'] for r in per_run)} 段**")
    n_timeout = sum(r["timeout"] is True for r in per_run)
    n_unknown_timeout = sum(r["timeout"] is None and not r["ok"] for r in per_run)
    P(f"- 逾時：**{n_timeout} 段**"
      + (f"；另有 {n_unknown_timeout} 個舊 FAIL 缺 `leg_timeout_s`，無法判定是否逾時"
         if n_unknown_timeout else ""))
    P("- 真值幾何淨空來自 PhysX 車身盒擴張重疊查詢；舊資料沒有此欄時顯示 `—`，"
      "不以光達距離代填。\n")

    groups = defaultdict(list)
    for r in per_run:
        groups[(r["model"], r["route"], r["density"], r["scenario"])].append(r)
    for route in sorted({r["route"] for r in per_run}):
        P(f"## 路線 {route}\n")
        P("| 模型 | " + ("密度 | " if stress else "")
          + "情境 | 成功趟 | 成功率 | 逾時段 | 卡住段 | 平均行駛 s（含逾時） | 平均路徑 m | 最小真值淨空 m |")
        P("|---|" + ("---|" if stress else "") + "---|---:|---:|---:|---:|---:|---:|---:|")
        for model in sorted({r["model"] for r in per_run}):
            densities = sorted({r["density"] for r in per_run}) if stress else [""]
            for density in densities:
                for scen in ("static", "dynamic", "mixed"):
                    rows = groups.get((model, route, density, scen), [])
                    if not rows:
                        continue
                    mean = lambda xs: sum(xs) / len(xs) if xs else float("nan")
                    drives = [r["drive"] for r in rows if r["drive"] is not None]
                    clear = [r["truth_clearance_m"] for r in rows
                             if r["truth_clearance_m"] is not None]
                    lower = [r["truth_clearance_lower_bound_m"] for r in rows
                             if r["truth_clearance_lower_bound_m"] is not None]
                    clear_cell = (f"{min(clear):.2f}" if clear else
                                  f"≥{min(lower):.2f}" if lower else "—")
                    by_run = defaultdict(list)
                    for r in rows:
                        by_run[r["tag"]].append(r)
                    success_runs = sum(len(v) == 2 and all(x["ok"] for x in v)
                                       for v in by_run.values())
                    success_pct = 100.0 * success_runs / len(by_run)
                    P(f"| {model} | " + (f"{density} | " if stress else "")
                      + f"{SCENARIO_ZH[scen]} | {success_runs}/{len(by_run)} | "
                      f"{success_pct:.1f}% | "
                      f"{sum(r['timeout'] is True for r in rows)} | "
                      f"{sum(r['stuck_segments'] for r in rows)} | "
                      f"{mean(drives):.1f} | {mean([r['dist'] for r in rows]):.1f} | "
                      f"{clear_cell} |")
        P("")
    P("行駛時間取逐段 CSV 的最後時間戳；不要用舊 `nav.log` 的原耗時。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
