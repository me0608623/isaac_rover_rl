"""相機避牆用的 2D 牆面格網（純 numpy，不依賴 Isaac，可離線測）。

為什麼不只用 PhysX 射線（2026-09-24）：
    斜前方相機加了 PhysX 避牆之後，sa4r2_c27_mixed_run02 仍有 64 幀全黑 ——
    車旁的門/內凹處只有**外觀網格、沒有碰撞體**，射線直接穿過去，相機照樣
    埋進牆後的黑色空間。相機拍的是**算圖幾何**，判斷「會不會進牆」也該用
    算圖幾何。

作法：在相機會出現的高度取幾個水平切面，把建築三角網格與切面相交得到
牆的線段，畫進 2D 格子。之後每幀在格子上沿射線找第一個牆格。
"""

from __future__ import annotations

import math

import numpy as np

#: 格子解析度（m）。牆厚通常 >= 10 cm，5 cm 足以擋住射線且不會太大。
CELL_M = 0.05
#: 取切面的高度（離地，m）。涵蓋斜前方 1.7 m、車後 2.0 m 相機與其視錐下緣。
SLICE_HEIGHTS_M = (0.9, 1.3, 1.7, 2.0, 2.3)


def slice_segments(tris: np.ndarray, z: float) -> np.ndarray:
    """三角形 ``(N, 3, 3)`` 與水平面 ``z`` 的交線段 → ``(M, 2, 2)``（只取 xy）。

    剛好貼在平面上的頂點視為在上方（避免同一條邊被算兩次）。
    """
    if len(tris) == 0:
        return np.zeros((0, 2, 2))
    s = tris[:, :, 2] - z                          # (N, 3)
    above = s >= 0
    n_above = above.sum(1)
    cross = (n_above == 1) | (n_above == 2)
    t = tris[cross]
    s = s[cross]
    ab = above[cross]
    pts = []
    for i, j in ((0, 1), (1, 2), (2, 0)):
        e = ab[:, i] != ab[:, j]
        denom = s[:, i] - s[:, j]
        u = np.where(e, s[:, i] / np.where(denom == 0, 1, denom), np.nan)
        p = t[:, i, :2] + (t[:, j, :2] - t[:, i, :2]) * u[:, None]
        pts.append(np.where(e[:, None], p, np.nan))
    p = np.stack(pts, 1)                           # (K, 3, 2)，恰有兩個非 NaN
    ok = ~np.isnan(p[:, :, 0])
    order = np.argsort(~ok, axis=1, kind="stable")[:, :2]
    seg = np.take_along_axis(p, order[:, :, None], axis=1)
    return seg


class WallGrid:
    """世界座標 xy 的佔據格子。"""

    def __init__(self, xmin: float, ymin: float, nx: int, ny: int,
                 cell: float = CELL_M):
        self.x0, self.y0, self.cell = xmin, ymin, cell
        self.occ = np.zeros((nx, ny), dtype=bool)

    @classmethod
    def from_segments(cls, segs: np.ndarray, cell: float = CELL_M, pad: float = 1.0):
        if len(segs) == 0:
            return cls(0.0, 0.0, 1, 1, cell)
        lo = segs.reshape(-1, 2).min(0) - pad
        hi = segs.reshape(-1, 2).max(0) + pad
        nx = int(math.ceil((hi[0] - lo[0]) / cell)) + 1
        ny = int(math.ceil((hi[1] - lo[1]) / cell)) + 1
        g = cls(float(lo[0]), float(lo[1]), nx, ny, cell)
        g.add_segments(segs)
        return g

    def add_segments(self, segs: np.ndarray) -> None:
        """把線段畫進格子（每段依長度取樣，間距半格）。"""
        if len(segs) == 0:
            return
        a, b = segs[:, 0], segs[:, 1]
        n = np.maximum(1, np.ceil(np.linalg.norm(b - a, axis=1) / (self.cell * 0.5))).astype(int)
        # 分塊避免一次展開太大
        for lo in range(0, len(segs), 20000):
            sl = slice(lo, lo + 20000)
            nn = n[sl]
            idx = np.repeat(np.arange(len(nn)), nn + 1)
            k = np.concatenate([np.arange(m + 1) for m in nn]) / np.repeat(nn, nn + 1)
            p = a[sl][idx] + (b[sl][idx] - a[sl][idx]) * k[:, None]
            self._mark(p)

    def _mark(self, p: np.ndarray) -> None:
        i = np.floor((p[:, 0] - self.x0) / self.cell).astype(int)
        j = np.floor((p[:, 1] - self.y0) / self.cell).astype(int)
        ok = (i >= 0) & (j >= 0) & (i < self.occ.shape[0]) & (j < self.occ.shape[1])
        self.occ[i[ok], j[ok]] = True

    def dilated(self, radius: float) -> "WallGrid":
        """回傳把牆往外加厚 ``radius`` 的新格網（圓盤膨脹）。

        用途：鏡頭「周圍」有沒有牆 —— 在加厚後的格網上，鏡頭點落在佔據格
        就代表離牆不到 ``radius``（牆在旁邊或斜後方也算，射線抓不到的那種）。
        """
        g = WallGrid(self.x0, self.y0, *self.occ.shape, cell=self.cell)
        r = int(math.ceil(radius / self.cell))
        occ = self.occ
        out = occ.copy()
        nx, ny = occ.shape
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj > r * r or (di == 0 and dj == 0):
                    continue
                src = occ[max(0, -di):nx - max(0, di), max(0, -dj):ny - max(0, dj)]
                out[max(0, di):nx - max(0, -di), max(0, dj):ny - max(0, -dj)] |= src
        g.occ = out
        return g

    def first_entry(self, ox: float, oy: float, dx: float, dy: float, length: float):
        """沿射線找「第一次**進入**佔據區」的距離（起點若已在佔據區，先走出去才算）。

        車貼著牆走時，車本身就在加厚範圍內；不跳過的話鏡頭會永遠被拉到最近。
        """
        n = math.hypot(dx, dy)
        if n < 1e-9 or length <= 0:
            return None
        ux, uy = dx / n, dy / n
        step = self.cell * 0.5
        k = 0.0
        inside = self.occupied(ox, oy)
        while k <= length:
            occ = self.occupied(ox + ux * k, oy + uy * k)
            if inside and not occ:
                inside = False
            elif not inside and occ:
                return k
            k += step
        return None

    def occupied(self, x: float, y: float) -> bool:
        i = int(math.floor((x - self.x0) / self.cell))
        j = int(math.floor((y - self.y0) / self.cell))
        if 0 <= i < self.occ.shape[0] and 0 <= j < self.occ.shape[1]:
            return bool(self.occ[i, j])
        return False

    def first_hit(self, ox: float, oy: float, dx: float, dy: float,
                  length: float, skip: float = 0.0):
        """沿 xy 方向 ``(dx, dy)``（不必是單位向量）走 ``length``（水平長度），
        回傳第一個牆格的水平距離，沒有回 None。``skip`` 內的格子不算
        （車自己站的位置若貼牆，不要把相機一開始就拉死）。"""
        n = math.hypot(dx, dy)
        if n < 1e-9 or length <= 0:
            return None
        ux, uy = dx / n, dy / n
        step = self.cell * 0.5
        k = max(skip, 0.0)
        while k <= length:
            if self.occupied(ox + ux * k, oy + uy * k):
                return k
            k += step
        return None


def triangles_from_mesh(points: np.ndarray, counts, indices) -> np.ndarray:
    """USD 多邊形（扇形三角化）→ ``(N, 3, 3)``。"""
    counts = np.asarray(counts, dtype=int)
    indices = np.asarray(indices, dtype=int)
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    tri = []
    for c in np.unique(counts):
        if c < 3:
            continue
        st = starts[counts == c]
        for k in range(1, c - 1):
            tri.append(np.stack([indices[st], indices[st + k], indices[st + k + 1]], 1))
    if not tri:
        return np.zeros((0, 3, 3))
    t = np.concatenate(tri)
    return points[t]
