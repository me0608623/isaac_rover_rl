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
import re
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from browse_names import ARM_DIR, ARM_ZH, CAMERA_ZH, run_label, video_name
from run_layout import (BROWSE_DIRNAME, MODEL_DIR_PREFIX,
                        model_dir_name, run_dirs)
from scene_counts import counts_from_log

MAIN_DIRNAME = "01_主批次_三模型正式錄影"
RAW_DIRNAME = "05_每趟原始資料_bag與CSV"

#: 每個模型資料夾裡**再放一份**自己的中文影片索引。
#: 理由：使用者會直接點進 `recordings/模型sa4r2/`，而那底下是 12 個英文
#: tag 資料夾（tag 不能改名，見 `browse_names` 檔頭）。在原地給一份中文索引，
#: 走到哪裡都看得懂。兩處由**同一支程式**產生，不會走樣。
MODEL_INDEX_DIRNAME = "00_中文影片"


def _route_of(meta) -> str:
    """run.json → 路線 key。舊錄影沒有 route_key，就從 route 的終點推。"""
    if meta.get("route_key"):
        return meta["route_key"]
    r = meta.get("route") or []
    return r[1] if len(r) >= 2 else ""


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


#: 這支程式產生的區塊都以「兩位數字 + _」開頭（`01_主批次…`、`00_中文影片`）。
#: 重建時只清這些，**使用者自己開的資料夾一律保留** ——
#: 2026-09-23 使用者在 `00_影片總覽/` 裡開了 `上傳/` 放要給人的片子，
#: 原本的 `shutil.rmtree(browse)` 下次重建就會把它整個刪掉。
_MANAGED = re.compile(r"^\d\d_")


def _clear_managed(parent: Path) -> None:
    """清掉 ``parent`` 底下由這支程式產生的區塊，其餘不動。"""
    if not parent.is_dir():
        return
    for child in sorted(parent.iterdir()):
        if child.is_dir() and not child.is_symlink() and _MANAGED.match(child.name):
            shutil.rmtree(child)


def _link(target: Path, link: Path) -> None:
    """影片用 **hard link**，資料夾用相對符號連結。

    ⚠ 影片曾經也用符號連結，結果使用者在檔案總管裡把它複製到別的資料夾之後
    「打不開」—— 複製出來的還是捷徑，而捷徑裡寫的是相對路徑
    （`../../../模型sa4r2/...`），換了層數就指到不存在的地方。
    hard link 在檔案總管裡就是一個普通檔案：複製它 = 複製真內容，
    而且和本體共用同一份資料、不佔額外空間。

    跨檔案系統時 hard link 會失敗（例如索引放在別顆硬碟），那時退回符號連結。
    """
    link.parent.mkdir(parents=True, exist_ok=True)
    if link.is_symlink() or link.exists():
        link.unlink()
    if target.is_dir():
        link.symlink_to(os.path.relpath(target.resolve(), link.parent.resolve()))
        return
    try:
        os.link(target, link)
    except OSError:
        link.symlink_to(os.path.relpath(target.resolve(), link.parent.resolve()))


def _runs(root: Path):
    """回傳 ``(run_dir, meta)``。「什麼算一趟」的定義在 `run_layout`。"""
    return [(d, json.loads((d / "run.json").read_text())) for d in run_dirs(root)]


def build(root: Path, arm_roots: dict[str, Path]) -> dict[str, int]:
    """重建索引樹，回傳 ``{區塊: 連結數}``。"""
    browse = root / BROWSE_DIRNAME
    _clear_managed(browse)
    stats: dict[str, int] = {}

    n = 0
    unknown = []
    for mdir in sorted(root.glob(f"{MODEL_DIR_PREFIX}*")):
        _clear_managed(mdir)
    for run_dir, meta in _runs(root):
        scen, idx = meta["scenario"], int(meta.get("run_index", 0))
        cnt = _counts(run_dir)
        if cnt is None:
            unknown.append(run_dir.name)
        mode = meta.get("crowd_mode")
        rk = _route_of(meta)
        model = meta.get("model") or meta["tag"].split("_", 1)[0]
        for mp4 in sorted((run_dir / "video").glob("*.mp4")):
            cam = mp4.stem.rsplit("_", 1)[-1]
            if cam not in CAMERA_ZH:
                continue
            zh = video_name(scen, idx, cam, cnt, mode, rk)
            _link(mp4, browse / MAIN_DIRNAME / f"模型{model}" / zh)
            # 原地再放一份：使用者會直接點進 recordings/模型xxx/
            _link(mp4, root / model_dir_name(model) / MODEL_INDEX_DIRNAME / zh)
            n += 1
        _link(run_dir, browse / RAW_DIRNAME
              / f"模型{model}_{run_label(scen, idx, cnt, mode, rk)}")
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
            rk = _route_of(meta)
            for mp4 in sorted((run_dir / "video").glob("*.mp4")):
                cam = mp4.stem.rsplit("_", 1)[-1]
                if cam not in CAMERA_ZH:
                    continue
                _link(mp4, browse / block
                      / video_name(scen, idx, cam, cnt, mode, rk))
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
    for mdir in sorted(root.glob(f"{MODEL_DIR_PREFIX}*")):
        k = len(list((mdir / MODEL_INDEX_DIRNAME).glob("*.mp4"))) \
            if (mdir / MODEL_INDEX_DIRNAME).is_dir() else 0
        print(f"  {mdir.name}/{MODEL_INDEX_DIRNAME}  {k:4d} 個（原地索引）")
    if total == 0:
        print("⚠ 一個連結都沒建 —— 檢查 root 路徑", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
