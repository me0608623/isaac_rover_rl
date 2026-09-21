"""從 NDT 地圖生成「USD 缺少的結構」補丁幾何（純函數）。

為什麼要補：NDT 的地圖是**真實世界掃出來的**，模擬的點雲是從 USD 模型
raycast 出來的。實測走廊段有 **25.5% 的地圖結構 USD 完全沒有**，且集中在
1.3~2.3 m 的天花板層（管線/風管/燈具/門框上緣）。

走廊是兩道平行長牆，牆面光禿禿就沒有**沿走廊方向**的特徵 —— 前進一公尺
與原地不動看起來幾乎一樣，NDT 因此估不出縱向位移（孔徑問題）。實測導航
失敗時「車走了 4.7 m，NDT 只認為移動 0.5 m」正是這個徵狀。

為什麼從地圖生成而不是手工建模：地圖**就是**真實結構的記錄，直接體素化
塞回 USD，補出來的幾何定義上就與地圖一致，不會引入新的模型-地圖落差。

⚠ 但這也是方法論上的限制：補進去的結構是用**同一份要對齊的地圖**推導的，
嚴格說不是獨立的環境建模。若因此 NDT 變好，無法區分「模擬變真實」與
「把答案抄給它」。論文若引用需標註。
"""

from __future__ import annotations

import numpy as np

#: 天花板層（map frame z）。只補這一層的理由：
#: 96% 的缺失在此，且車高約 1.0 m 撞不到 —— 純粹給定位用，不影響導航行為。
CEILING_BAND = (1.2, 2.4)

#: 體素大小 m。0.25 是實測折衷：0.15 → 8278 塊（PhysX 吃不消），
#: 0.40 → 1582 塊（特徵變粗）。0.25 給 3419 塊，沿走廊每 2 m 有 92~486 個特徵。
VOXEL_M = 0.25

#: 判「USD 沒有」的水平距離門檻 m。
MISSING_TOL_M = 0.30


def voxelize_missing(map_pts: np.ndarray, usd_pts: np.ndarray,
                     voxel: float = VOXEL_M,
                     tol: float = MISSING_TOL_M) -> np.ndarray:
    """回傳「地圖有、USD 沒有」的體素索引 (N, 3)。

    ⚠ 容差只比 **xy**：同一根柱子在不同高度都算「有建模」。若連 z 一起比，
    USD 的面積取樣點沒剛好落在同一高度就會被誤判成缺失，補出一堆假結構。
    """
    from scipy.spatial import cKDTree

    if len(map_pts) == 0:
        return np.zeros((0, 3), dtype=int)
    if len(usd_pts) == 0:
        miss = map_pts
    else:
        d, _ = cKDTree(usd_pts[:, :2]).query(map_pts[:, :2])
        miss = map_pts[d > tol]
    if len(miss) == 0:
        return np.zeros((0, 3), dtype=int)
    cells = np.floor(miss / voxel).astype(int)
    return np.unique(cells, axis=0)


def voxel_centers(cells: np.ndarray, voxel: float = VOXEL_M) -> np.ndarray:
    """體素索引 → 中心座標。"""
    return cells.astype(float) * voxel + voxel / 2.0


def boxes_to_mesh(centers: np.ndarray, size: float):
    """把一堆方塊合併成**單一**網格。

    回傳 ``(points, faceVertexCounts, faceVertexIndices)``。

    為什麼合併而不是每塊一個 prim：3419 個 prim 會讓 USD 載入與 PhysX
    場景查詢都變重（prim 數是線性成本），而單一靜態三角網格用 BVH，
    raycast 成本幾乎與面數無關（對數級）。也只需要設一次 visibility。
    """
    if len(centers) == 0:
        return [], [], []
    h = size / 2.0
    # 立方體 8 頂點的相對位置（順序固定，下面的面索引依賴它）
    corner = np.array([(-h, -h, -h), (+h, -h, -h), (+h, +h, -h), (-h, +h, -h),
                       (-h, -h, +h), (+h, -h, +h), (+h, +h, +h), (-h, +h, +h)])
    # 6 面 × 2 三角形，繞序一致朝外
    faces = [(0, 3, 2), (0, 2, 1),          # -z
             (4, 5, 6), (4, 6, 7),          # +z
             (0, 1, 5), (0, 5, 4),          # -y
             (2, 3, 7), (2, 7, 6),          # +y
             (1, 2, 6), (1, 6, 5),          # +x
             (0, 4, 7), (0, 7, 3)]          # -x
    pts: list[tuple[float, float, float]] = []
    idx: list[int] = []
    for k, c in enumerate(centers):
        base = k * 8
        pts.extend(tuple(c + v) for v in corner)
        for f in faces:
            idx.extend(base + i for i in f)
    counts = [3] * (len(centers) * 12)
    return pts, counts, idx
