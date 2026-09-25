#!/usr/bin/env python3
"""三視角合成：俯視、車後、斜前方＋資訊欄放在同一個 1920×1080 畫面（2026-09-25）。

    ┌──────────┬──────────┐
    │  俯視     │  車後     │
    ├──────────┼──────────┤
    │  斜前方   │  資訊欄   │
    └──────────┴──────────┘

輸出到各趟 ``video/<tag>_combined.mp4``，make_browse_tree 會一併放進中文索引。
資訊欄的數字全部讀既有資料（行駛時間表、該趟 log 的場景數量、擦撞紀錄），
不在這裡重算 —— 同一個數字只有一個出處。

用法：
    python3 scripts/make_combined.py recordings            # 全部
    python3 scripts/make_combined.py recordings sa4r2_c27_mixed_run04
"""

from __future__ import annotations

import csv
import json
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from browse_names import CAMERA_ZH, SCENARIO_ZH, counts_label, route_dir
from run_layout import run_dirs
from scene_counts import counts_from_log

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
W, H = 960, 540
#: 資訊欄底色與字色（深色底，三個畫面都是亮色場景，資訊欄要一眼分得開）
PANEL_BG = "0x1b2320"
ROUTE_ZH = {"c27": "路線A  c28 ⇄ c27", "c36": "路線B  c28 ⇄ c36"}


def _driving(csv_path: Path):
    """``{(model, route, scenario, run): {"去": row, "回": row}}``。"""
    out: dict = {}
    if not csv_path.exists():
        return out
    for r in csv.DictReader(open(csv_path)):
        k = (r["model"], r["route"], r["scenario"], int(r["run"]))
        out.setdefault(k, {})[r["leg"]] = r
    return out


def info_lines(meta: dict, run_dir: Path, drive: dict) -> list[str]:
    model = meta.get("model") or meta["tag"].split("_", 1)[0]
    route = meta.get("route_key") or ""
    scen, idx = meta["scenario"], int(meta["run_index"])
    log = run_dir / "isaac_nav.log"
    cnt = counts_from_log(log.read_text(errors="replace")) if log.exists() else None
    coll = json.loads((run_dir / "collisions_summary.json").read_text())
    n_coll = sum(v.get("episodes", 0) for v in coll.get("by_category", {}).values())
    lines = [
        f"模型 {model}",
        ROUTE_ZH.get(route, route_dir(route)),
        f"{SCENARIO_ZH[scen]}  第 {idx} 趟",
        f"場上 {counts_label(cnt)}（靜止 / 走動）",
        "",
    ]
    legs = drive.get((model, route, scen, idx), {})
    for leg in ("去", "回"):
        r = legs.get(leg)
        if r:
            lines.append(f"{leg}程  {float(r['drive']):5.1f} s   {float(r['dist']):4.1f} m")
    lines += [f"真的碰到  {n_coll} 次", "", "ORCA 行人 · 速度上限 0.7"]
    return lines


def build(run_dir: Path, drive: dict, force: bool = False) -> Path | None:
    meta = json.loads((run_dir / "run.json").read_text())
    tag = meta["tag"]
    v = run_dir / "video"
    src = {c: v / f"{tag}_{c}.mp4" for c in ("topdown", "chase", "oblique")}
    if not all(p.exists() for p in src.values()):
        print(f"⚠ {tag} 缺視角，略過", file=sys.stderr)
        return None
    out = v / f"{tag}_combined.mp4"
    if out.exists() and not force and out.stat().st_mtime > max(
            p.stat().st_mtime for p in src.values()):
        return out
    with tempfile.TemporaryDirectory() as td:
        info = Path(td) / "info.txt"
        info.write_text("\n".join(info_lines(meta, run_dir, drive)))
        labels = {}
        for c in src:
            labels[c] = Path(td) / f"{c}.txt"
            labels[c].write_text(CAMERA_ZH[c])

        def lab(i, c):
            return (f"[{i}:v]scale={W}:{H},drawtext=fontfile={FONT}:textfile={labels[c]}:"
                    f"x=14:y=12:fontsize=30:fontcolor=white:box=1:boxcolor=0x000000aa:"
                    f"boxborderw=8[{c}]")
        fc = ";".join([
            lab(0, "topdown"), lab(1, "chase"), lab(2, "oblique"),
            f"color=c={PANEL_BG}:s={W}x{H}:r=30,drawtext=fontfile={FONT}:textfile={info}:"
            f"x=48:y=36:fontsize=29:fontcolor=white:line_spacing=10,"
            f"drawtext=fontfile={FONT}:text='%{{eif\\:t\\:d}} s':x=w-tw-40:y=h-th-30:"
            f"fontsize=28:fontcolor=0x7fd6bd[info]",
            "[topdown][chase]hstack[top]",
            "[oblique][info]hstack=shortest=1[bot]",
            "[top][bot]vstack=shortest=1[out]",
        ])
        cmd = ["ffmpeg", "-y", "-nostdin", "-v", "error"]
        for c in ("topdown", "chase", "oblique"):
            cmd += ["-i", str(src[c])]
        cmd += ["-filter_complex", fc, "-map", "[out]", "-r", "30",
                "-c:v", "h264_nvenc", "-preset", "p5", "-cq", "23",
                "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-an",
                "-f", "mp4", str(out) + ".new"]
        subprocess.run(cmd, check=True)
        Path(str(out) + ".new").replace(out)
    return out


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "recordings")
    only = sys.argv[2] if len(sys.argv) > 2 else ""
    drive = _driving(Path("reports/driving_time.csv"))
    n = 0
    for d in run_dirs(root):
        if only and only not in d.name:
            continue
        out = build(d, drive, force=bool(only))
        if out:
            n += 1
            print(f"{d.name} → {out.name}", flush=True)
    print(f"完成 {n} 支三視角合成")
    return 0 if n else 1


if __name__ == "__main__":
    raise SystemExit(main())
