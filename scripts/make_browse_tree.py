#!/usr/bin/env python3
"""在 recordings/ 下建一層中文符號連結索引，方便挑影片。

真資料夾名（`sa4r2_mixed_run04`）保持不動 —— 理由見 `browse_names.py` 檔頭。
這棵樹只有連結，不複製任何位元，重建隨時可重建。

產出：

    recordings/00_影片總覽/
        01_主批次_三模型正式錄影/模型sa4r2/靜動態混合_第4趟_障礙6行人8_俯視.mp4
        02_對照_行人走固定路線/...
        05_每趟原始資料_bag與CSV/模型sa4r2_靜動態混合_第4趟_障礙6行人8 → ../../../sa4r2_mixed_run04

用法：
    python3 scripts/make_browse_tree.py
"""

from __future__ import annotations

import json
import os
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from browse_names import ARM_DIR, ARM_ZH, CAMERA_ZH, run_label, video_name
from run_layout import BROWSE_DIRNAME, run_dirs
from scene_counts import counts_from_log

MAIN_DIRNAME = "01_主批次_三模型正式錄影"
RAW_DIRNAME = "05_每趟原始資料_bag與CSV"


def _counts(run_dir: Path):
    """回傳該趟實際在場的 ``(靜態數, 動態數)``，讀不出來回 None。

    ⚠ 一定要讀該趟自己的 `isaac_nav.log`，不可查 `scene_variants` 的計畫表 ——
    執行期會停用太靠近車 / 站在 routing 點上 / 擠在一起的角色，
    把計畫值寫進檔名等於讓檔名有機會說謊。
    """
    log = run_dir / "isaac_nav.log"
    if not log.exists():
        return None
    return counts_from_log(log.read_text(errors="replace"))


def _link(target: Path, link: Path) -> None:
    """建相對路徑的符號連結（整個 recordings/ 搬走也不會斷）。"""
    link.parent.mkdir(parents=True, exist_ok=True)
    if link.is_symlink() or link.exists():
        link.unlink()
    link.symlink_to(os.path.relpath(target.resolve(), link.parent.resolve()))


def _runs(root: Path):
    """回傳 ``(run_dir, meta)``。「什麼算一趟」的定義在 `run_layout`。"""
    return [(d, json.loads((d / "run.json").read_text())) for d in run_dirs(root)]


def build(root: Path, arm_roots: dict[str, Path]) -> dict[str, int]:
    """重建索引樹，回傳 ``{區塊: 連結數}``。"""
    browse = root / BROWSE_DIRNAME
    if browse.exists():
        shutil.rmtree(browse)                 # 整棵重建：改名後不留孤兒連結
    stats: dict[str, int] = {}

    n = 0
    unknown = []
    for run_dir, meta in _runs(root):
        scen, idx = meta["scenario"], int(meta.get("run_index", 0))
        cnt = _counts(run_dir)
        if cnt is None:
            unknown.append(run_dir.name)
        mode = meta.get("crowd_mode")
        model = meta.get("model") or meta["tag"].split("_", 1)[0]
        for mp4 in sorted((run_dir / "video").glob("*.mp4")):
            cam = mp4.stem.rsplit("_", 1)[-1]
            if cam not in CAMERA_ZH:
                continue
            _link(mp4, browse / MAIN_DIRNAME / f"模型{model}"
                  / video_name(scen, idx, cam, cnt, mode))
            n += 1
        _link(run_dir, browse / RAW_DIRNAME
              / f"模型{model}_{run_label(scen, idx, cnt, mode)}")
    stats[MAIN_DIRNAME] = n
    if unknown:
        print(f"⚠ {len(unknown)} 趟讀不出場景數量，檔名標「數量不明」："
              f"{unknown[:5]}", file=sys.stderr)

    for i, (arm, arm_root) in enumerate(sorted(arm_roots.items()), start=2):
        if not arm_root.is_dir():
            continue
        block = f"{i:02d}_{ARM_ZH[arm]}"
        m = 0
        for run_dir, meta in _runs(arm_root):
            scen, idx = meta["scenario"], int(meta.get("run_index", 0))
            cnt = _counts(run_dir)
            mode = meta.get("crowd_mode")
            for mp4 in sorted((run_dir / "video").glob("*.mp4")):
                cam = mp4.stem.rsplit("_", 1)[-1]
                if cam not in CAMERA_ZH:
                    continue
                _link(mp4, browse / block
                      / video_name(scen, idx, cam, cnt, mode))
                m += 1
        stats[block] = m
    return stats


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "recordings")
    abl = Path(sys.argv[2] if len(sys.argv) > 2 else "recordings_abl")
    arms = {a: abl / ARM_DIR[a] for a in ARM_DIR}
    stats = build(root, arms)
    total = sum(stats.values())
    print(f"已重建 {root / BROWSE_DIRNAME}")
    for k, v in stats.items():
        print(f"  {k:28s} {v:4d} 個影片連結")
    print(f"  合計 {total} 個影片連結"
          f" + {len(_runs(root))} 個原始資料夾連結")
    if total == 0:
        print("⚠ 一個連結都沒建 —— 檢查 root 路徑", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
