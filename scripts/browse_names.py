"""把 run tag 翻成看得懂的中文名（純字串運算，不碰檔案系統）。

為什麼不直接把 36 個資料夾改成中文名：tag 被寫進 `run.json` 的 `tag` 欄、
`nav/<tag>_leg?_*.csv`、`video/<tag>_*.mp4`，而 `record_batch.sh ONLY=<tag>`、
`make_readme.py`、`check_recordings.py`、`compare_arms.py`、`nearest_source.py`
全都靠它對應。改名會讓「重跑某一趟」與所有分析失效，而且重跑批次又會把
舊名字的資料夾生回來。所以真名保持原樣，另外做一層中文符號連結索引 ——
順便可以讓**每個影片檔**也有中文名（單純改資料夾名做不到這件事）。
"""

from __future__ import annotations

SCENARIO_ZH = {
    "static": "靜態障礙",
    "dynamic": "動態行人",
    "mixed": "靜動態混合",
}

CAMERA_ZH = {
    "topdown": "俯視",
    "chase": "車後",
    "oblique": "斜前方",
}

ARM_ZH = {
    "crowd_path": "對照_行人走固定路線",
    "speed_0p6": "對照_速度0.6",
    "speed_1p0": "對照_速度1.0",
}


def scene_load(scenario: str, n_obstacles: int, n_walkers: int) -> str:
    """回傳該趟「場上有什麼」的短標。

    三個情境放的東西不一樣，寫成同一種格式會誤導：
    dynamic 沒有靜態障礙，static 的行人不走動。
    """
    if scenario == "static":
        return f"障礙{n_obstacles}"
    if scenario == "dynamic":
        return f"行人{n_walkers}"
    return f"障礙{n_obstacles}行人{n_walkers}"


def run_label(scenario: str, run_index: int, n_obstacles: int,
              n_walkers: int) -> str:
    """一趟的中文標籤，例如 ``靜動態混合_第4趟_障礙6行人8``。"""
    if scenario not in SCENARIO_ZH:
        raise ValueError(f"未知情境 {scenario!r}")
    return (f"{SCENARIO_ZH[scenario]}_第{run_index}趟_"
            f"{scene_load(scenario, n_obstacles, n_walkers)}")


def video_name(scenario: str, run_index: int, camera: str,
               n_obstacles: int, n_walkers: int) -> str:
    """一個影片檔的中文名。"""
    if camera not in CAMERA_ZH:
        raise ValueError(f"未知視角 {camera!r}")
    return (f"{run_label(scenario, run_index, n_obstacles, n_walkers)}_"
            f"{CAMERA_ZH[camera]}.mp4")
