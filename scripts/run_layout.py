"""`recordings/` 的版面：一趟的資料夾在哪、怎麼找出全部。

單一定義的理由：有六支程式各自用 `root.iterdir()` 找「一趟」。
2026-09-23 把 36 趟按模型分到子資料夾之後，那六支會全部找到 0 趟
（模型資料夾裡沒有 `run.json`）—— 而且**不會報錯**，只會安靜地輸出空表。
所以「什麼算一趟」只能有一處定義。

版面：

    recordings/
        模型sa4r2/sa4r2_static_run01/run.json      ← 一趟
        模型sa4r3/...
        00_影片總覽/                               ← 索引樹，裡面是符號連結
        _作廢批次_只留紀錄/                        ← 存檔

    recordings_abl/crowd_path/sa4r2_static_run01/  ← 對照組沒有模型層

所以 `run_dirs()` 要同時看 root 底下與**模型子資料夾**底下兩層。
"""

from __future__ import annotations

from pathlib import Path

#: 模型資料夾的前綴。`模型sa4r2` 比 `sa4r2` 一眼看得出那是分類層。
MODEL_DIR_PREFIX = "模型"

#: 中文索引樹的資料夾名（`make_browse_tree.BROWSE_DIRNAME` 與此一致）。
#: ⚠ 一定要排除：索引樹裡是**指向各趟的符號連結**，跟著收的話每趟會被算兩次。
BROWSE_DIRNAME = "00_影片總覽"


def model_dir_name(model: str) -> str:
    """模型的分類資料夾名，例如 ``sa4r2`` → ``模型sa4r2``。"""
    if not model:
        raise ValueError("模型名是空的")
    return f"{MODEL_DIR_PREFIX}{model}"


def model_from_dir_name(name: str) -> str | None:
    """反向：``模型sa4r2`` → ``sa4r2``；不是模型資料夾回 None。"""
    if name.startswith(MODEL_DIR_PREFIX) and len(name) > len(MODEL_DIR_PREFIX):
        return name[len(MODEL_DIR_PREFIX):]
    return None


def _skip(p: Path) -> bool:
    """這個資料夾是不是「不該當成一趟、也不該往裡面找」。"""
    return (p.is_symlink() or not p.is_dir()
            or p.name.startswith("_") or p.name == BROWSE_DIRNAME)


def run_dirs(root) -> list[Path]:
    """回傳 ``root`` 底下所有「一趟」的資料夾（含模型子層），依路徑排序。

    判準是**裡面有 `run.json`**，不是名字長相 —— 名字規則改過好幾次。
    """
    root = Path(root)
    if not root.is_dir():
        return []
    out = []
    for p in sorted(root.iterdir()):
        if _skip(p):
            continue
        if (p / "run.json").exists():
            out.append(p)
            continue
        for q in sorted(p.iterdir()):          # 模型子資料夾往下一層
            if not _skip(q) and (q / "run.json").exists():
                out.append(q)
    return out


def run_dir_for(root, model: str, tag: str) -> Path:
    """一趟該放哪：``root/模型<model>/<tag>``。"""
    return Path(root) / model_dir_name(model) / tag
