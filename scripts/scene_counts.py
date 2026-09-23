"""從每一趟的 `isaac_nav.log` 數出「場上有幾個靜態、幾個動態」。

為什麼不直接查 `scene_variants.variant(i)` 的表：表是**計畫**，log 是**實況**。
`run_isaac_sim` 會停用太靠近車、站在 routing 點上、或擠在一起的角色
（log 印「停用(...)」）。目前錄的 36 趟剛好都沒發生，但把計畫值寫進檔名
等於讓檔名有機會說謊 —— 檔名就是文件。

三個情境實際在場上的東西不一樣（2026-09-23 逐幀查證）：

    情境      障礙圓柱/箱   站立人物            走動人物
    static    啟用          在障礙位置          停放在變體指定的安全位置（**也是靜態**）
    dynamic   全部關閉      在障礙位置(仍在場)  ORCA
    mixed     啟用          在障礙位置          ORCA

⚠ 站立人物是**擺到 person 障礙圓柱的位置上**，所以障礙啟用時兩者在同一點，
算「靜態物件數」要扣掉這個重疊，否則 mixed 會從 6 變成 11。
static 的停放行人（log「停放行人 N 個」）不疊在任何障礙上，**不扣**。
"""

from __future__ import annotations

import re

# 路線欄位是 2026-09-23 加的（「情境 static／路線 c27／變體 run4」），舊 log 沒有
_SCENE = re.compile(r"情境 (\w+)／(?:路線 \w+／)?變體 run(\d+)：靜態障礙 (\d+)/(\d+) 啟用")
# 驅動器建好**之後**才停用的角色：「程序化步態：N 人」的 N 仍含他們。
# 這行只在 N>0 時印，讀不到就是 0（不是缺資料）。
_OFF_LATE = re.compile(r"停用\(障礙關閉，站立人物不該存在\) (\d+) 個")
_PARK_FAIL = re.compile(r"⚠ 停放失敗 ")
_GAIT = re.compile(
    r"程序化步態：(\d+) 人 / \d+ 個擺動關節 / (\d+) 人沿路徑移動"
    r"\s*站立人物擺位 (\d+) 個")


def parse_log(text: str):
    """回傳 ``{scenario, run_index, obstacles_enabled, chars, walking, standing}``。

    兩行有任何一行讀不到就回 None —— **不要湊一個預設值**，
    那會讓檔名上的數字看起來一樣可信。
    """
    m1 = _SCENE.search(text)
    m2 = _GAIT.search(text)
    if not (m1 and m2):
        return None
    return {
        "scenario": m1.group(1),
        "run_index": int(m1.group(2)),
        "obstacles_enabled": int(m1.group(3)),
        # ⚠ 2026-09-24 dynamic 被誤判成「靜2」：4 人裡有 2 個是事後停用的站立人物
        "chars": (int(m2.group(1))
                  - sum(int(x) for x in _OFF_LATE.findall(text))
                  - len(_PARK_FAIL.findall(text))),
        "walking": int(m2.group(2)),
        "standing": int(m2.group(3)),
    }


def counts(info) -> tuple[int, int]:
    """``info`` → ``(靜態物件數, 動態物件數)``。

    靜態 = 啟用的障礙 + 不會動的角色 − 兩者重疊（站立人物疊在 person 障礙上）。
    動態 = 沿路徑移動的角色。
    """
    obs = info["obstacles_enabled"]
    still = info["chars"] - info["walking"]
    overlap = info["standing"] if obs else 0
    return (obs + still - overlap, info["walking"])


def counts_from_log(text: str):
    """一步到底；讀不到回 None。"""
    info = parse_log(text)
    return counts(info) if info else None
