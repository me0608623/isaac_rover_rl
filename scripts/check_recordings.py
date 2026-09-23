#!/usr/bin/env python3
"""錄影產物健檢：抓「全黑」與「幀數不對」。

為什麼需要：這個專案在「RTX Real-Time 算出全黑」上卡了很久，而全黑的影片
檔案大小看起來正常、ffmpeg 也不報錯 —— 只有真的去看像素才知道。
108 段影片不可能一段一段點開看，所以自動抓。

用法：  python3 scripts/check_recordings.py recordings
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

#: 平均亮度低於此值視為「沒算出光照」。
#: 先前全黑時的實測值：整張 0.00，只剩背景天空時 1.00。正常畫面是 110~190。
DARK_MEAN_LUMA = 5.0

#: 幀數容許的短缺比例。BasicWriter 收尾常少幾幀（2582 → 2578）。
FRAME_SLACK = 0.02


def expected_frames(duration_s: float, fps: int) -> int:
    return int(round(duration_s * fps))


def frame_count_ok(actual: int, expected: int) -> bool:
    if actual <= 0:
        return False
    return actual >= expected * (1.0 - FRAME_SLACK)


def is_too_dark(mean_luma) -> bool:
    """亮度低於門檻視為沒算出光照。

    ⚠ ``None`` 代表**量不到**，不是「全黑」。2026-09-23 踩到：
    signalstats 的輸出沒解析成功 → 回 0.0 → 108 段全部被判全黑，
    但實際亮度是 111~206。讓失敗的量測偽裝成壞結果，比沒量更糟。
    """
    if mean_luma is None:
        return False
    return mean_luma < DARK_MEAN_LUMA


def is_unmeasured(mean_luma) -> bool:
    return mean_luma is None


def summarise(rows):
    def _bad(r):
        return (not frame_count_ok(r["frames"], r["expected"])
                or is_too_dark(r["mean"]))

    bad = {r["tag"] for r in rows if _bad(r)}
    unmeasured = {r["tag"] for r in rows if is_unmeasured(r["mean"])}
    ok = sum(1 for r in rows if not _bad(r) and not is_unmeasured(r["mean"]))
    return {"total": len(rows), "ok": ok, "bad_tags": sorted(bad),
            "unmeasured_tags": sorted(unmeasured),
            "all_ok": len(rows) > 0 and ok == len(rows)}


def _probe(mp4: Path):
    """回傳 ``(幀數, 平均亮度)``。亮度用 ffmpeg 的 signalstats 取樣幾幀。"""
    n = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-count_frames", "-show_entries", "stream=nb_read_frames",
         "-of", "csv=p=0", str(mp4)],
        capture_output=True, text=True, timeout=600).stdout.strip()
    frames = int(n) if n.isdigit() else 0
    # 抽幾幀算平均亮度。
    # ⚠ 不要用 signalstats + metadata=print 去解 stderr —— 2026-09-23 實測
    #   解不到值就回 0.0，害 108 段全部被誤判全黑（實際亮度 111~206）。
    #   改成把畫面縮小後以 rawvideo 灰階倒進 stdout，自己算，沒有解析可言。
    step = max(1, frames // 9) if frames else 1
    W, H = 160, 90
    proc = subprocess.run(
        ["ffmpeg", "-nostdin", "-v", "error", "-i", str(mp4),
         "-vf", f"select='not(mod(n\\,{step}))',scale={W}:{H}",
         "-vsync", "0", "-pix_fmt", "gray", "-f", "rawvideo", "-"],
        capture_output=True, timeout=900)
    buf = proc.stdout
    if not buf or len(buf) < W * H:
        return frames, None          # 量不到 ≠ 全黑
    import numpy as np
    a = np.frombuffer(buf[: (len(buf) // (W * H)) * W * H], dtype=np.uint8)
    return frames, float(a.mean())


def main(root: Path) -> int:
    import json

    rows = []
    for d in sorted(p for p in root.iterdir() if p.is_dir() and not p.name.startswith("_")):
        meta_path = d / "run.json"
        if not meta_path.exists():
            print(f"  {d.name}: 沒有 run.json，跳過")
            continue
        meta = json.loads(meta_path.read_text())
        fps = int(meta.get("fps", 30))
        ft = d / "frame_times.csv"
        exp = 0
        if ft.exists():
            lines = [l for l in ft.read_text().splitlines() if l and not l.startswith("frame")]
            exp = len(lines)
        for mp4 in sorted((d / "video").glob("*.mp4")):
            frames, mean = _probe(mp4)
            cam = mp4.stem.rsplit("_", 1)[-1]
            rows.append({"tag": d.name, "camera": cam, "frames": frames,
                         "expected": exp or frames,
                         "mean": None if mean is None else round(mean, 2)})
            flag = ""
            if not frame_count_ok(frames, exp or frames):
                flag += f"  ⚠ 幀數 {frames} / 預期 {exp}"
            if is_unmeasured(mean):
                flag += "  ⚠ 亮度量不到（工具問題，不代表影片壞）"
            elif is_too_dark(mean):
                flag += f"  ⚠ 全黑（平均亮度 {mean:.2f}）"
            shown = "  n/a  " if mean is None else f"{mean:7.2f}"
            print(f"  {d.name:30s} {cam:9s} {frames:6d} 幀  亮度 {shown}{flag}")
    s = summarise(rows)
    print(f"\n合計 {s['total']} 段，正常 {s['ok']} 段")
    if s["bad_tags"]:
        print(f"  ⚠ 有問題的趟：{', '.join(s['bad_tags'])}")
    if s["unmeasured_tags"]:
        print(f"  ⚠ 亮度量不到（工具問題）：{', '.join(s['unmeasured_tags'])}")
    return 0 if s["all_ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main(Path(sys.argv[1] if len(sys.argv) > 1 else "recordings")))
