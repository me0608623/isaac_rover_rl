"""地圖補丁幾何的測試（純函數，不需要 Isaac / PCD）。"""

from __future__ import annotations

import numpy as np
import pytest

from map_patch import boxes_to_mesh, voxel_centers, voxelize_missing


def test_finds_points_with_no_nearby_reference():
    """地圖有、USD 沒有 → 該點算缺失。"""
    m = np.array([[0.0, 0, 1.5], [10.0, 0, 1.5]])
    u = np.array([[0.0, 0, 1.5]])
    cells = voxelize_missing(m, u, voxel=0.25, tol=0.3)
    assert len(cells) == 1                      # 只有 (10,0) 那顆算缺失


def test_points_close_to_reference_are_not_missing():
    m = np.array([[0.0, 0, 1.5], [0.2, 0, 1.5]])
    u = np.array([[0.0, 0, 1.5]])
    assert len(voxelize_missing(m, u, voxel=0.25, tol=0.3)) == 0


def test_tolerance_is_horizontal_only():
    """★ 容差只比 xy：同一根柱子在不同高度都算「有建模」，
    否則牆面只要 USD 的取樣點沒落在同一高度就會被誤判成缺失。"""
    m = np.array([[0.0, 0, 2.2]])
    u = np.array([[0.0, 0, 0.5]])               # 同一 xy、差 1.7 m 高
    assert len(voxelize_missing(m, u, voxel=0.25, tol=0.3)) == 0


def test_multiple_points_in_one_voxel_collapse():
    m = np.array([[5.0, 0, 1.5], [5.05, 0, 1.5], [5.1, 0, 1.5]])
    u = np.zeros((1, 3))
    assert len(voxelize_missing(m, u, voxel=0.25, tol=0.3)) == 1


def test_voxel_centers_land_in_the_middle():
    cells = np.array([[0, 0, 0], [1, 2, 3]])
    c = voxel_centers(cells, 0.25)
    assert c[0] == pytest.approx([0.125, 0.125, 0.125])
    assert c[1] == pytest.approx([0.375, 0.625, 0.875])


def test_mesh_has_twelve_triangles_per_box():
    """每個方塊 6 面 × 2 三角形。面數錯了 PhysX cook 出來會有破洞。"""
    pts, counts, idx = boxes_to_mesh(np.array([[0.0, 0, 0]]), 0.25)
    assert len(counts) == 12
    assert all(c == 3 for c in counts)
    assert len(idx) == 36
    assert len(pts) == 8


def test_mesh_scales_with_box_count():
    pts, counts, idx = boxes_to_mesh(np.array([[0.0, 0, 0], [1.0, 0, 0]]), 0.25)
    assert len(pts) == 16 and len(counts) == 24 and len(idx) == 72


def test_mesh_box_has_the_right_size():
    pts, _, _ = boxes_to_mesh(np.array([[0.0, 0, 0]]), 0.4)
    a = np.array(pts)
    assert a[:, 0].max() - a[:, 0].min() == pytest.approx(0.4)
    assert a[:, 2].max() - a[:, 2].min() == pytest.approx(0.4)


def test_mesh_indices_stay_in_range():
    """★ 索引越界會讓 USD 靜默產生壞網格。"""
    pts, _, idx = boxes_to_mesh(np.random.default_rng(0).random((20, 3)) * 10, 0.25)
    assert min(idx) >= 0 and max(idx) < len(pts)


def test_empty_input_gives_empty_mesh():
    pts, counts, idx = boxes_to_mesh(np.zeros((0, 3)), 0.25)
    assert pts == [] and counts == [] and idx == []
