"""把一趟翻成看得懂的中文名（純字串運算，不碰檔案系統）。

為什麼不直接把各趟的資料夾改成中文名：tag 被寫進 `run.json` 的 `tag` 欄、
`nav/<tag>_leg?_*.csv`、`video/<tag>_*.mp4`，而 `record_batch.sh ONLY=<tag>`
與所有分析程式都靠它對應。改名會讓「重跑某一趟」與全部分析失效，而且重跑
批次又會把舊名字生回來。所以各趟的真名保持原樣，另外做一層中文符號連結
索引 —— 順便可以讓**每個影片檔**也有中文名（單純改資料夾名做不到）。

檔名格式：``{情境}_{行人模式}_第N趟_靜{S}動{D}_{視角}.mp4``，例如

    混合_ORCA互動_第4趟_靜6動8_俯視.mp4
    純靜態_不走動_第4趟_靜14動0_車後.mp4
    混合_固定路線_第4趟_靜6動8_斜前方.mp4      ← 對照組（非 ORCA）

靜/動的數量一律**從該趟自己的 log 數出來**（見 `scene_counts`），不是查計畫表。
"""

from __future__ import annotations

#: 情境的中文名。
#:
#: ⚠ `dynamic` **不可以叫「純動態」**。2026-09-23 使用者指出
#: `純動態_ORCA互動_第1趟_靜3動2` 自我矛盾：說「純動態」卻有 3 個靜態。
#: 事實是 `dynamic` 只關掉障礙圓柱/箱，`run_isaac_sim` 的 `place_standing`
#: **不看 `obstacles_enabled`**，照樣把 3~5 個站立人物擺到那些位置上 ——
#: 所以場上仍有靜止的人形障礙。名字不能宣稱它沒有。
#:
#: `static` 叫「純靜態」是名副其實的（動態數確實是 0）。
#: 這種不對稱反映的是事實，比對稱但說謊好。
SCENARIO_ZH = {
    "static": "純靜態",
    "dynamic": "動態",
    "mixed": "混合",
}

CAMERA_ZH = {
    "topdown": "俯視",
    "chase": "車後",
    "oblique": "斜前方",
}

#: 行人的走法。`orca` = RVO2 互動避讓（會閃避彼此與車）；`path` = 沿固定路線來回。
CROWD_MODE_ZH = {
    "orca": "ORCA互動",
    "path": "固定路線",
}

#: 沒有任何人在走的時候用這個標，不管 run.json 記的 crowd_mode 是什麼。
NO_WALK_ZH = "不走動"

#: 對照組的資料夾名（`recordings_abl/` 底下）。編號讓它們照設計順序排。
ARM_DIR = {
    "crowd_path": "對照1_行人走固定路線_非ORCA",
    "speed_0p6": "對照2_速度0.6",
    "speed_1p0": "對照3_速度1.0",
}

#: 索引樹裡對照組區塊的名字。
ARM_ZH = {
    "crowd_path": "對照_行人走固定路線_非ORCA",
    "speed_0p6": "對照_速度0.6",
    "speed_1p0": "對照_速度1.0",
}

#: 數量讀不出來時用的字樣 —— 寧可寫「不明」也不要填 0。
UNKNOWN_COUNTS = "數量不明"


def crowd_mode_label(crowd_mode, n_dynamic: int) -> str:
    """行人走法的標籤。

    ⚠ `static` 情境的 `run.json` 照樣記 `crowd_mode: orca`（那是 CLI 參數），
    但 `walks_enabled=False` 所以**沒有人在走**。只看 crowd_mode 會把
    純靜態的片子標成「ORCA互動」，檔名自己說謊。所以先看實際動態數。
    """
    if n_dynamic == 0:
        return NO_WALK_ZH
    return CROWD_MODE_ZH.get(crowd_mode, str(crowd_mode))


def counts_label(counts) -> str:
    """``(靜態, 動態)`` → ``靜6動8``；``None`` → ``數量不明``。"""
    if counts is None:
        return UNKNOWN_COUNTS
    return f"靜{counts[0]}動{counts[1]}"


def route_label(route) -> str:
    """``c27`` → ``路線c27``；空的回空字串（舊錄影沒有路線維度）。

    ⚠ 2026-09-23 加入第二條路線：兩條路線同一個情境、同一趟的影片檔名
      原本會一模一樣，放進同一個資料夾就互相蓋掉。
    """
    return f"路線{route}_" if route else ""


def run_label(scenario: str, run_index: int, counts, crowd_mode, route="") -> str:
    """一趟的中文標籤，例如 ``路線c27_混合_ORCA互動_第4趟_靜6動8``。"""
    if scenario not in SCENARIO_ZH:
        raise ValueError(f"未知情境 {scenario!r}")
    n_dyn = counts[1] if counts else 0
    return (f"{route_label(route)}{SCENARIO_ZH[scenario]}_"
            f"{crowd_mode_label(crowd_mode, n_dyn)}"
            f"_第{run_index}趟_{counts_label(counts)}")


def video_name(scenario: str, run_index: int, camera: str, counts,
               crowd_mode, route="") -> str:
    """一個影片檔的中文名。"""
    if camera not in CAMERA_ZH:
        raise ValueError(f"未知視角 {camera!r}")
    return (f"{run_label(scenario, run_index, counts, crowd_mode, route)}_"
            f"{CAMERA_ZH[camera]}.mp4")
