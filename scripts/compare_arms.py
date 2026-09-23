#!/usr/bin/env python3
"""把對照組與基準線做**成對**比較。

為什麼要成對：每一趟的場景（障礙與行人的數量與位置）是由 run_index 的
亂數種子決定的，同一個 (情境, 難度) 在所有批次裡**逐項相同**。所以拿
同一格互比，場景的隨機性被消掉，剩下的差異只能來自那個變數。

⚠ 配對必須用 (scenario, run_index)，不能用 tag 字串 —— 對照組的目錄前綴
不同（recordings_abl/<組名>/…），用 tag 配會全部配不到而且不報錯。

⚠ 每格只有 1 個樣本（4 趟是 4 個不同難度，不是同一場景重複 4 次），
所以沒有「同一場景的變異數」。這裡只報成對差值與符號檢定，不報 t 檢定。

用法：
    python3 scripts/compare_arms.py recordings recordings_abl/crowd_path
"""

from __future__ import annotations

import json
import math
import statistics as st
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))


def load_runs(root: Path, model_filter: str | None = None):
    """讀一個批次目錄下每趟的彙總。"""
    from make_readme import cell_summary, parse_nav_table

    out = []
    for d in sorted(p for p in root.iterdir() if p.is_dir() and not p.name.startswith("_")):
        mp = d / "run.json"
        if not mp.exists():
            continue
        meta = json.loads(mp.read_text())
        if model_filter and meta.get("model") != model_filter:
            continue
        nav = (d / "nav.log").read_text(errors="replace") if (d / "nav.log").exists() else ""
        legs = parse_nav_table(nav)
        c = cell_summary(legs)
        out.append({
            "tag": meta["tag"], "scenario": meta["scenario"],
            "run_index": meta.get("run_index", 0),
            "speed_rate": meta.get("speed_rate"),
            "crowd_mode": meta.get("crowd_mode"),
            "arrived": c["arrived"], "legs": c["legs"],
            "min_range_m": c["min_range_m"],
            "collision_frames": c["collision_frames"],
            "seconds": c["seconds"],
        })
    return out


def pair_runs(base, arm):
    """依 (情境, 難度) 配對。配不到的丟掉，不硬湊。"""
    key = lambda r: (r["scenario"], r["run_index"])
    a = {key(r): r for r in arm}
    return [(b, a[key(b)]) for b in base if key(b) in a]


def paired_delta(pairs, field: str, higher_is_better: bool = True):
    """回傳成對差值（對照組 − 基準線）。沒有配對時 median 回 None。

    ⚠ ``higher_is_better`` 不可省：碰撞幀與耗時是**越小越好**，
    把「變大」算成「變好」會讓結論完全講反。
    2026-09-23 第一版就少了這個參數，差點報成「固定路線行人讓碰撞幀變好 6 次」，
    而實際是那 6 次從 0 幀惡化到 1~25 幀。
    """
    ds = []
    for b, a in pairs:
        if b.get(field) is None or a.get(field) is None:
            continue
        ds.append(a[field] - b[field])
    if not ds:
        return {"n": 0, "deltas": [], "median": None, "better": 0, "worse": 0}
    if higher_is_better:
        better = sum(1 for d in ds if d > 0)
        worse = sum(1 for d in ds if d < 0)
    else:
        better = sum(1 for d in ds if d < 0)
        worse = sum(1 for d in ds if d > 0)
    return {"n": len(ds), "deltas": ds, "median": st.median(ds),
            "better": better, "worse": worse}


def sign_test_p(better: int, worse: int):
    """雙尾符號檢定的 p 值。有效樣本 <5 時回 None（不該報 p 值）。"""
    n = better + worse
    if n < 5:
        return None
    k = min(better, worse)
    tail = sum(math.comb(n, i) for i in range(0, k + 1)) / (2 ** n)
    return min(1.0, 2.0 * tail)


def _fmt(v, unit="", nd=2):
    return "—" if v is None else f"{v:+.{nd}f}{unit}"


def main(base_root: Path, arm_root: Path) -> int:
    base = load_runs(base_root, model_filter="sa4r2")
    arm = load_runs(arm_root, model_filter="sa4r2")
    pairs = pair_runs(base, arm)
    if not pairs:
        print("配不到任何一對 —— 檢查 run.json 的 scenario / run_index", file=sys.stderr)
        return 1

    a0 = arm[0]
    print(f"基準線：{base_root}（sa4r2，{len(base)} 趟，"
          f"speed_rate {base[0]['speed_rate']}，行人 {base[0]['crowd_mode']}）")
    print(f"對照組：{arm_root}（{len(arm)} 趟，"
          f"speed_rate {a0['speed_rate']}，行人 {a0['crowd_mode']}）")
    print(f"成對配到 {len(pairs)} 對\n")

    hdr = (f"{'情境':<9s}{'難度':<7s}{'抵達':^13s}{'最近障礙 m':^17s}"
           f"{'碰撞幀':^15s}{'每段秒數':^17s}")
    print(hdr)
    print("-" * 78)
    for b, a in sorted(pairs, key=lambda p: (p[0]["scenario"], p[0]["run_index"])):
        arr = f"{b['arrived']}/{b['legs']} → {a['arrived']}/{a['legs']}"
        rng = f"{b['min_range_m']:.2f} → {a['min_range_m']:.2f}"
        col = f"{b['collision_frames']} → {a['collision_frames']}"
        sec = f"{b['seconds']:.1f} → {a['seconds']:.1f}"
        mark = ""
        if a["arrived"] < b["arrived"]:
            mark = "  ← 變成未達"
        elif a["collision_frames"] > b["collision_frames"] + 5:
            mark = "  ← 碰撞幀大增"
        print(f"{b['scenario']:<9s}run{b['run_index']:02d}   "
              f"{arr:^13s}{rng:^17s}{col:^15s}{sec:^17s}{mark}")

    print()
    for field, unit, nd, good, hib in (
            ("min_range_m", " m", 2, "越大越好", True),
            ("collision_frames", " 幀", 0, "越小越好", False),
            ("seconds", " s", 1, "越小越快", False),
            ("arrived", " 段", 0, "越大越好", True)):
        d = paired_delta(pairs, field, higher_is_better=hib)
        p = sign_test_p(d["better"], d["worse"])
        pstr = "樣本太少" if p is None else f"p={p:.3f}"
        print(f"  {field:18s}（{good}）中位差 {_fmt(d['median'], unit, nd)}　"
              f"變好 {d['better']} / 變差 {d['worse']} / 持平 "
              f"{d['n']-d['better']-d['worse']}　符號檢定 {pstr}")
    print("\n  ⚠ 每格只有 1 個樣本（4 趟是 4 個不同難度，不是重複試驗），")
    print("    所以沒有同場景的變異數估計；差值小的時候無法斷定不是雜訊。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(Path(sys.argv[1]), Path(sys.argv[2])))
