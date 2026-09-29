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
from collections import deque
from pathlib import Path

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

    @classmethod
    def from_ros_map(cls, pgm, yml, *, unknown_is_occupied: bool = True):
        """從 ROS ``map_server`` 的 PGM/YAML 建立 map-frame 格網。

        ``WallGrid`` 內部以 ``occ[x_index, y_index]`` 儲存，PGM 則以左上角為
        原點、陣列第一維往下，因此這裡會同時轉置並翻轉 y。對可通行性採保守
        規則：機率介於 ``free_thresh`` 與 ``occupied_thresh`` 的未知格預設也
        視為佔據，避免把尚未建圖的區域當成繞路捷徑。
        """
        import yaml

        cfg = yaml.safe_load(Path(yml).read_text())
        img = _read_pgm(pgm)
        negate = int(cfg.get("negate", 0))
        occ_prob = (img.astype(np.float32) / 255.0 if negate
                    else (255.0 - img.astype(np.float32)) / 255.0)
        if unknown_is_occupied:
            occ_img = occ_prob >= float(cfg["free_thresh"])
        else:
            occ_img = occ_prob > float(cfg["occupied_thresh"])
        origin = cfg["origin"]
        g = cls(float(origin[0]), float(origin[1]), img.shape[1], img.shape[0],
                float(cfg["resolution"]))
        g.occ = occ_img[::-1, :].T.copy()
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

    def add_disk(self, x: float, y: float, radius: float) -> None:
        """把世界座標的實心圓畫進格網。"""
        i0, i1, j0, j1 = self._index_bounds(x - radius, x + radius,
                                             y - radius, y + radius)
        if i0 >= i1 or j0 >= j1:
            return
        xs = self.x0 + (np.arange(i0, i1) + 0.5) * self.cell
        ys = self.y0 + (np.arange(j0, j1) + 0.5) * self.cell
        self.occ[i0:i1, j0:j1] |= ((xs[:, None] - x) ** 2
                                   + (ys[None, :] - y) ** 2 <= radius ** 2)

    def add_oriented_box(self, x: float, y: float, size_x: float, size_y: float,
                         yaw: float = 0.0) -> None:
        """把中心在 ``(x,y)`` 的旋轉長方形畫進格網（``yaw`` 單位 rad）。"""
        hx, hy = size_x / 2.0, size_y / 2.0
        c, s = math.cos(yaw), math.sin(yaw)
        ex, ey = abs(c) * hx + abs(s) * hy, abs(s) * hx + abs(c) * hy
        i0, i1, j0, j1 = self._index_bounds(x - ex, x + ex, y - ey, y + ey)
        if i0 >= i1 or j0 >= j1:
            return
        xs = self.x0 + (np.arange(i0, i1) + 0.5) * self.cell - x
        ys = self.y0 + (np.arange(j0, j1) + 0.5) * self.cell - y
        # 世界 → 方箱局部座標（旋轉 -yaw）。
        lx = c * xs[:, None] + s * ys[None, :]
        ly = -s * xs[:, None] + c * ys[None, :]
        self.occ[i0:i1, j0:j1] |= (np.abs(lx) <= hx) & (np.abs(ly) <= hy)

    def _index_bounds(self, xmin: float, xmax: float, ymin: float, ymax: float):
        """世界座標 bbox → 夾在格網內的半開 index 範圍。"""
        i0 = max(0, int(math.floor((xmin - self.x0) / self.cell)))
        i1 = min(self.occ.shape[0], int(math.ceil((xmax - self.x0) / self.cell)) + 1)
        j0 = max(0, int(math.floor((ymin - self.y0) / self.cell)))
        j1 = min(self.occ.shape[1], int(math.ceil((ymax - self.y0) / self.cell)) + 1)
        return i0, i1, j0, j1

    def dilated(self, radius: float) -> "WallGrid":
        """回傳把牆往外加厚 ``radius`` 的新格網（圓盤膨脹）。

        用途：鏡頭「周圍」有沒有牆 —— 在加厚後的格網上，鏡頭點落在佔據格
        就代表離牆不到 ``radius``（牆在旁邊或斜後方也算，射線抓不到的那種）。
        """
        g = WallGrid(self.x0, self.y0, *self.occ.shape, cell=self.cell)
        r = int(math.ceil(radius / self.cell))
        occ = self.occ
        if r <= 0:
            g.occ = occ.copy()
            return g
        yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
        footprint = xx * xx + yy * yy <= r * r
        try:
            from scipy import ndimage
            g.occ = ndimage.binary_dilation(occ, structure=footprint)
            return g
        except ImportError:  # 相機執行環境若沒有 scipy，保留純 numpy 路徑。
            pass
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

    def has_path(self, start, goal, *, diagonal: bool = True) -> bool:
        """自由格中是否存在 ``start → goal`` 路徑。

        這是幾何可通行檢查，不是路徑規劃器：呼叫端應先把牆與障礙依所需
        通道半寬膨脹，再查剩下的自由格是否連通。端點在格網外或佔據格時回
        ``False``，不會偷偷把端點吸到附近自由格。
        """
        a, b = self._world_index(*start), self._world_index(*goal)
        if a is None or b is None or self.occ[a] or self.occ[b]:
            return False
        if a == b:
            return True
        steps = ((1, 0), (-1, 0), (0, 1), (0, -1))
        if diagonal:
            steps += ((1, 1), (1, -1), (-1, 1), (-1, -1))
        seen = np.zeros_like(self.occ, dtype=bool)
        seen[a] = True
        q = deque([a])
        nx, ny = self.occ.shape
        while q:
            i, j = q.popleft()
            for di, dj in steps:
                ni, nj = i + di, j + dj
                if not (0 <= ni < nx and 0 <= nj < ny):
                    continue
                if seen[ni, nj] or self.occ[ni, nj]:
                    continue
                # 斜走不可從兩個牆角的縫隙穿過。
                if di and dj and (self.occ[i + di, j] or self.occ[i, j + dj]):
                    continue
                if (ni, nj) == b:
                    return True
                seen[ni, nj] = True
                q.append((ni, nj))
        return False

    def _world_index(self, x: float, y: float):
        i = int(math.floor((x - self.x0) / self.cell))
        j = int(math.floor((y - self.y0) / self.cell))
        if 0 <= i < self.occ.shape[0] and 0 <= j < self.occ.shape[1]:
            return (i, j)
        return None

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


def _read_pgm(path) -> np.ndarray:
    """讀取 binary PGM（P5）；允許標頭任意位置出現 ``#`` 註解。"""
    raw = Path(path).read_bytes()
    if raw[:2] != b"P5":
        raise ValueError(f"{path} 不是 binary PGM（開頭 {raw[:2]!r}）")
    i, vals = 2, []
    while len(vals) < 3:
        if i >= len(raw):
            raise ValueError(f"{path} 的 PGM 標頭不完整")
        c = raw[i:i + 1]
        if c == b"#":
            i = raw.index(b"\n", i) + 1
        elif c.isspace():
            i += 1
        else:
            j = i
            while j < len(raw) and not raw[j:j + 1].isspace():
                j += 1
            vals.append(int(raw[i:j]))
            i = j
    while i < len(raw) and raw[i:i + 1].isspace():
        i += 1
    w, h, maxval = vals
    if maxval > 255:
        raise ValueError(f"{path} 的 maxval={maxval}，只支援 8-bit PGM")
    return np.frombuffer(raw, dtype=np.uint8, count=w * h, offset=i).reshape(h, w)
