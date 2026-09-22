"""錄影情境（靜態 / 動態 / 靜動態混合）的設定表。

純資料，不碰 stage —— 這樣設定本身可以單獨測試，
實際去 stage 上開關 prim 的那幾行留在 run_isaac_sim。

三個情境要拍的東西：
  static   走廊裡有靜態障礙（推車、立柱狀物），行人站著不動
  dynamic  行人沿路徑走動，靜態障礙全部關掉
  mixed    兩者都有（= 目前驗證過的正式組態）
"""

from __future__ import annotations

from dataclasses import dataclass

SCENARIO_NAMES: tuple[str, ...] = ("static", "dynamic", "mixed")


@dataclass(frozen=True)
class ScenarioConfig:
    """一個錄影情境。"""

    name: str
    #: /World/SimObstacles 底下的靜態障礙要不要存在
    obstacles_enabled: bool
    #: 行人要不要沿路徑走（False 時仍然套基礎站姿，不會是 T-pose）
    walks_enabled: bool


_TABLE = {
    "static": ScenarioConfig("static", obstacles_enabled=True, walks_enabled=False),
    "dynamic": ScenarioConfig("dynamic", obstacles_enabled=False, walks_enabled=True),
    "mixed": ScenarioConfig("mixed", obstacles_enabled=True, walks_enabled=True),
}


def scenario_config(name: str) -> ScenarioConfig:
    """查表。名字打錯要**大聲**報錯。

    靜靜地退回預設值的話，36 段影片會有一批拍成一樣的內容，
    而且要等全部跑完才看得出來。
    """
    try:
        return _TABLE[name]
    except KeyError:
        raise ValueError(
            f"未知的錄影情境 {name!r}，可用的是 {', '.join(SCENARIO_NAMES)}") from None
