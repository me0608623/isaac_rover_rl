"""走廊靜態障礙可以用的 Isaac 官方道具（純資料，不碰 stage）。

⚠⚠ 光達是 **PhysX 光達**（`isaacsim.sensors.physx.IsaacReadLidarPointCloud`
讀 `/velodyne/Lidar`），它對**物理碰撞體**做 raycast，**不看算圖網格**。
2026-09-23 我曾誤以為是 RTX 光達（stage 裡另外有一個沒被接到發佈節點的
`RTX_Lidar` prim），寫出「隱形圓柱光達看不到」的錯誤結論。已更正。

所以一個道具要被光達看到，必須同時滿足：
  1. **有碰撞體**，而且碰撞體涵蓋光達那一層（離地 0.93~1.93 m）
  2. 碰撞體夠高 —— 矮的東西光達從上面掃過去，等於不存在。
     這正是 finding_ndt_crop_min_z 查出來的導航失敗根因：
     「車撞上感知不到的 1 m 矮障礙物並卡死」。

做法：每個道具配一個**隱形的 bbox 碰撞盒**（光達、物理、ORCA、事後分析
四者用同一個幾何），並關掉道具自帶的三角網格碰撞體。

清單只收**可見帶內厚度 >= MIN_BAND_OVERLAP_M** 的。2026-09-23 從 CDN 下載
15 個 Office 道具實測 bbox，被刷掉的（光達看不到）：
  SM_Armchair 0.79 m、SM_MarkerBoard 0.90 m、SM_ReceptionStandA 0.70 m、
  SM_Printer 0.53 m、SM_Extinguisher 0.58 m、SM_Plant03 0.95 m（只擦到 0.02 m）、
  SM_ChairOffice 1.18 m（0.25 m）、SM_Partition 1.20 m（0.27 m，且厚度只有 4 cm）。

所有道具 metersPerUnit=1.0、upAxis=Z、原點在底部（z_min=0）—— 與 People
角色相同，而角色以 scale=1 參照進建物 stage 後尺寸正確，所以道具不需補縮放。
（建物 stage 標頭寫的是 metersPerUnit=0.01、upAxis=Y，與實際內容不符，不可信。）
"""

from __future__ import annotations

import math
from dataclasses import dataclass

PROP_BASE_URL = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
                 "/Assets/Isaac/5.1/Isaac/Environments/Office/Props")

#: 光達看得到的那一層（離地，m）。感測器離地 1.43 m，policy 與 monitor 都只留
#: 感測器座標 |z| <= 0.5。
LIDAR_BAND_M = (0.93, 1.93)

#: 道具碰撞體在可見帶內至少要有多厚（m）。太薄的只會偶爾被一兩條掃描線掃到。
MIN_BAND_OVERLAP_M = 0.4


@dataclass(frozen=True)
class PropSpec:
    """一個道具。bbox 用它**自己的**座標（公尺、Z 朝上、原點在底部）。"""

    name: str
    zh: str
    bbox_min: tuple[float, float, float]
    bbox_max: tuple[float, float, float]

    @property
    def url(self) -> str:
        return f"{PROP_BASE_URL}/{self.name}.usd"

    @property
    def size_x(self) -> float:
        return self.bbox_max[0] - self.bbox_min[0]

    @property
    def size_y(self) -> float:
        return self.bbox_max[1] - self.bbox_min[1]

    @property
    def height(self) -> float:
        return self.bbox_max[2] - self.bbox_min[2]

    @property
    def centre_xy(self) -> tuple[float, float]:
        """bbox 中心相對於道具原點的水平偏移。

        ⚠ 不是每個道具都置中：SM_Cupboard 的 x 是 -0.109~+0.310，
        中心偏 +0.10 m。碰撞盒要放在 bbox 中心，模型要反向補這個偏移。
        """
        return ((self.bbox_min[0] + self.bbox_max[0]) / 2.0,
                (self.bbox_min[1] + self.bbox_max[1]) / 2.0)

    @property
    def extent_radius(self) -> float:
        """水平外接圓半徑（m）。算與站點、與行人的淨空時用它。"""
        return math.hypot(self.size_x, self.size_y) / 2.0

    @property
    def long_axis_is_y(self) -> bool:
        return self.size_y > self.size_x


def band_overlap(height: float, band=LIDAR_BAND_M) -> float:
    """底部貼地、高 ``height`` 的東西在可見帶內有多厚（m）。"""
    return max(0.0, min(height, band[1]) - band[0])


#: 2026-09-23 實測（從 CDN 下載後用 usd-core 算 bbox）。
PROPS: tuple[PropSpec, ...] = (
    PropSpec("SM_Cupboard", "置物櫃", (-0.109, -0.920, 0.000), (0.310, 0.920, 2.220)),
    PropSpec("SM_Rack", "貨架", (-0.307, -0.612, 0.000), (0.307, 0.612, 1.829)),
    PropSpec("SM_Plant01", "大盆栽", (-0.648, -0.763, 0.000), (0.657, 0.778, 1.755)),
    PropSpec("SM_Plant02", "盆栽", (-0.413, -0.458, 0.000), (0.524, 0.550, 1.554)),
    PropSpec("SM_FileCabinet_01", "檔案櫃", (-0.337, -0.196, 0.000), (0.367, 0.196, 1.341)),
    PropSpec("SM_FileCabinet_02", "檔案櫃", (-0.337, -0.196, 0.000), (0.367, 0.196, 1.341)),
)

_BY_NAME = {p.name: p for p in PROPS}


def prop(name: str) -> PropSpec:
    """依名字取道具。名字打錯要大聲報錯。"""
    try:
        return _BY_NAME[name]
    except KeyError:
        raise ValueError(f"未知的道具 {name!r}，可用的是 "
                         f"{', '.join(_BY_NAME)}") from None


def yaw_along(heading: float, spec: PropSpec) -> float:
    """讓道具的**長邊平行走廊**（像靠牆擺）要轉的 yaw（rad，同 heading 的座標系）。"""
    return heading - math.pi / 2.0 if spec.long_axis_is_y else heading
