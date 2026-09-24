"""2D 牆面格網的測試（相機避牆用，2026-09-24）。"""

from __future__ import annotations

import numpy as np

from wall_grid import WallGrid, slice_segments, triangles_from_mesh


def _wall(x, y0, y1, z0=0.0, z1=3.0):
    """x = const 的一面直立牆（兩個三角形）。"""
    p = np.array([[x, y0, z0], [x, y1, z0], [x, y1, z1], [x, y0, z1]], float)
    return triangles_from_mesh(p, [4], [0, 1, 2, 3])


def test_vertical_wall_slices_into_one_segment_per_triangle():
    segs = slice_segments(_wall(2.0, -1.0, 1.0), 1.5)
    assert segs.shape == (2, 2, 2)
    assert np.allclose(segs[:, :, 0], 2.0)


def test_floor_triangles_below_the_slice_do_not_count():
    """★ 地板三角形整片在切面下方，不可變成牆 —— 否則相機到處都被擋。"""
    p = np.array([[-5, -5, 0], [5, -5, 0], [5, 5, 0], [-5, 5, 0]], float)
    assert len(slice_segments(triangles_from_mesh(p, [4], [0, 1, 2, 3]), 1.5)) == 0


def test_ray_stops_at_the_wall():
    g = WallGrid.from_segments(slice_segments(_wall(2.0, -1.0, 1.0), 1.5))
    hit = g.first_hit(0.0, 0.0, 1.0, 0.0, 3.2)
    assert hit is not None and 1.9 <= hit <= 2.05


def test_ray_that_misses_the_wall_is_clear():
    g = WallGrid.from_segments(slice_segments(_wall(2.0, -1.0, 1.0), 1.5))
    assert g.first_hit(0.0, 0.0, 0.0, 1.0, 3.2) is None       # 平行牆面
    assert g.first_hit(0.0, 0.0, 1.0, 0.0, 1.5) is None       # 還沒到牆


def test_wall_without_collider_is_what_this_catches():
    """★★ 這個格網就是為了「只有外觀、沒有碰撞體」的牆存在：
    輸入只是網格頂點，不看任何物理屬性。"""
    tris = _wall(-1.0, -3.0, 3.0)
    g = WallGrid.from_segments(slice_segments(tris, 1.7))
    assert g.first_hit(0.0, 0.0, -1.0, 0.0, 3.0) is not None
