"""道具清單的測試。"""

from __future__ import annotations

import math

import pytest

from props import (LIDAR_BAND_M, MIN_BAND_OVERLAP_M, PROPS, band_overlap, prop,
                   yaw_along)


def test_every_prop_is_tall_enough_for_the_lidar():
    """★★ 每個道具都要穿過光達那一層至少 MIN_BAND_OVERLAP_M。

    矮的東西光達從上面掃過去，等於不存在 —— finding_ndt_crop_min_z 查出的
    導航失敗根因就是「車撞上感知不到的 1 m 矮障礙物並卡死」。
    """
    for p in PROPS:
        assert band_overlap(p.height) >= MIN_BAND_OVERLAP_M, (
            f"{p.name} 高 {p.height:.2f} m，可見帶內只有 "
            f"{band_overlap(p.height):.2f} m")


def test_the_rejected_low_props_really_are_invisible():
    """★ 把刷掉的理由鎖住：這幾個高度實測過，光達看不到或只擦到邊。"""
    for name, h in (("SM_Armchair", 0.79), ("SM_MarkerBoard", 0.90),
                    ("SM_Printer", 0.53), ("SM_Extinguisher", 0.58),
                    ("SM_Plant03", 0.95), ("SM_ChairOffice", 1.18)):
        assert band_overlap(h) < MIN_BAND_OVERLAP_M, name
        assert name not in {p.name for p in PROPS}


def test_band_overlap():
    lo, hi = LIDAR_BAND_M
    assert band_overlap(0.5) == 0.0
    assert band_overlap(1.43) == pytest.approx(1.43 - lo)
    assert band_overlap(3.0) == pytest.approx(hi - lo)      # 高過帶子就是整條帶


def test_bbox_origin_is_on_the_floor():
    """★ 原點不在底部的話會懸空或陷進地板（角色就踩過這個坑：懸空 17~20 cm）。"""
    for p in PROPS:
        assert p.bbox_min[2] == pytest.approx(0.0, abs=1e-3), p.name


def test_off_centre_prop_reports_its_offset():
    """★ SM_Cupboard 的 bbox 中心不在原點（x 偏 +0.10 m）。
    碰撞盒擺在原點的話會跟模型錯開 10 cm。"""
    cx, cy = prop("SM_Cupboard").centre_xy
    assert cx == pytest.approx(0.1005, abs=1e-3)
    assert cy == pytest.approx(0.0, abs=1e-3)


def test_unknown_prop_raises():
    with pytest.raises(ValueError):
        prop("SM_DoesNotExist")


def test_yaw_puts_the_long_side_along_the_corridor():
    """長邊要平行走廊（靠牆擺），不能橫在走廊中間。"""
    heading = 0.3
    cup = prop("SM_Cupboard")               # 長邊是本地 y
    cab = prop("SM_FileCabinet_01")         # 長邊是本地 x
    assert cup.long_axis_is_y and not cab.long_axis_is_y
    # 本地長軸轉完之後要與 heading 同向
    y_axis_after = yaw_along(heading, cup) + math.pi / 2.0
    assert math.isclose(math.cos(y_axis_after - heading), 1.0, abs_tol=1e-9)
    assert yaw_along(heading, cab) == pytest.approx(heading)


def test_urls_point_at_the_office_props_folder():
    for p in PROPS:
        assert p.url.endswith(f"/Environments/Office/Props/{p.name}.usd")
