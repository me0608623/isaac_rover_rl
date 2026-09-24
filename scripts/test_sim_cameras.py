"""論文錄影用的三視角相機幾何測試。"""

from __future__ import annotations

import math

import pytest

from sim_cameras import CAMERAS, camera_pose, look_at_rotation


def test_three_angles_are_defined():
    names = {c.name for c in CAMERAS}
    assert names == {"topdown", "chase", "oblique"}


#: 角色最高點離地高度（Characters bbox 實測 world z 1.79，地板 -0.32）。
TALLEST_CHARACTER_H = 2.11


def test_topdown_clears_the_pedestrians():
    """★ 俯視相機要高過所有行人，否則會拍到後腦勺擋住車。

    這棟樓**沒有天花板**（相機朝正上方拍是均勻的天空背景），
    所以高度沒有上限；早先「天花板 3.65 m」是整棟樓 Mesh bbox 的
    最高點，不是走廊局部高度。
    """
    cam = next(c for c in CAMERAS if c.name == "topdown")
    pos, _ = camera_pose(cam, (1.0, 2.0), yaw=0.0, floor_z=0.0)
    assert pos[0] == pytest.approx(1.0, abs=0.5)
    assert pos[1] == pytest.approx(2.0, abs=0.5)
    assert pos[2] > TALLEST_CHARACTER_H + 1.0, "要明顯高過行人頭頂"


def test_topdown_framing_is_usable():
    """★ 俯視圖的地面涵蓋範圍要落在可用區間。

    太窄看不到周遭行人；太寬車（約 0.6 m）在 1280 px 寬的畫面裡
    會縮成幾十個像素，論文截圖看不出東西。
    """
    from sim_cameras import ground_coverage_m

    cam = next(c for c in CAMERAS if c.name == "topdown")
    w, h = ground_coverage_m(cam)
    assert 6.0 < w < 12.0, f"俯視涵蓋 {w:.1f} m 寬，超出可用區間"
    assert h > 4.0, f"縱向只涵蓋 {h:.1f} m，車一動就出框"


def test_side_cameras_stay_inside_the_corridor():
    """★ 側向位移太大，相機會埋進牆裡。

    2026-09-22 斜前方相機設側向 3.5 m，實拍畫面六成是白牆。
    """
    from sim_cameras import CORRIDOR_HALF_WIDTH_M

    for cam in CAMERAS:
        assert abs(cam.offset[1]) <= CORRIDOR_HALF_WIDTH_M, (
            f"{cam.name} 側向 {cam.offset[1]} m 超過走廊半寬 "
            f"{CORRIDOR_HALF_WIDTH_M} m，會拍到牆內")


def test_chase_looks_over_pedestrian_heads():
    """★ 車後視角若低於行人頭頂，起點人群密集時整台車會被擋住
    （2026-09-22 實拍：高 1.8 m 的畫面被三個行人塞滿）。"""
    cam = next(c for c in CAMERAS if c.name == "chase")
    assert cam.offset[2] >= 1.9, f"車後相機只有 {cam.offset[2]} m，會被行人擋住"


def test_topdown_orientation_ignores_robot_heading():
    """★ 俯視圖若跟著車轉，畫面會一直旋轉，看的人會暈。
    位置跟著車走，方向固定朝世界正下方。"""
    cam = next(c for c in CAMERAS if c.name == "topdown")
    (a0, ang0) = camera_pose(cam, (0.0, 0.0), yaw=0.0, floor_z=0.0)[1]
    (a1, ang1) = camera_pose(cam, (0.0, 0.0), yaw=math.pi / 2, floor_z=0.0)[1]
    # pytest.approx 不支援巢狀結構，軸與角要分開比
    assert a0 == pytest.approx(a1, abs=1e-9)
    assert ang0 == pytest.approx(ang1, abs=1e-9)


def test_chase_is_behind_the_robot():
    """車後視角：相機在車**後方**，車頭朝畫面深處。"""
    cam = next(c for c in CAMERAS if c.name == "chase")
    # 車朝 +x（yaw=0）→ 相機應在 -x 側
    pos, _ = camera_pose(cam, (0.0, 0.0), yaw=0.0, floor_z=0.0)
    assert pos[0] < -1.0, f"相機應在車後方，實際 x={pos[0]}"


def test_chase_follows_robot_heading():
    """★ 車後視角必須跟著車頭轉，否則車一轉彎就跑出畫面。"""
    cam = next(c for c in CAMERAS if c.name == "chase")
    p0, _ = camera_pose(cam, (0.0, 0.0), yaw=0.0, floor_z=0.0)
    p1, _ = camera_pose(cam, (0.0, 0.0), yaw=math.pi / 2, floor_z=0.0)
    # 車轉 90° → 相機也該繞到另一側
    assert p0[:2] != pytest.approx(p1[:2], abs=0.1)


def test_oblique_is_ahead_and_to_the_side():
    """斜前方旁觀：在車前方偏側邊，回頭看車。"""
    cam = next(c for c in CAMERAS if c.name == "oblique")
    pos, _ = camera_pose(cam, (0.0, 0.0), yaw=0.0, floor_z=0.0)
    assert pos[0] > 1.0, "應在車前方"
    assert abs(pos[1]) > 1.0, "應偏一側"


def test_all_cameras_stay_above_the_floor():
    for cam in CAMERAS:
        pos, _ = camera_pose(cam, (0.0, 0.0), yaw=0.3, floor_z=-0.32)
        assert pos[2] > -0.32 + 0.5, f"{cam.name} 相機貼地或穿地板"


def test_look_at_points_the_minus_z_axis():
    """★ USD 相機看向自身 -Z 軸。算錯會拍到反方向。"""
    from pxr import Gf
    r = look_at_rotation((0.0, 0.0, 5.0), (0.0, 0.0, 0.0))
    d = Gf.Rotation(*r).TransformDir(Gf.Vec3d(0, 0, -1))
    assert Gf.Vec3d(d) == pytest.approx((0.0, 0.0, -1.0), abs=1e-6)


def test_look_at_handles_horizontal_view():
    from pxr import Gf
    r = look_at_rotation((-5.0, 0.0, 1.0), (0.0, 0.0, 1.0))
    d = Gf.Rotation(*r).TransformDir(Gf.Vec3d(0, 0, -1))
    assert Gf.Vec3d(d) == pytest.approx((1.0, 0.0, 0.0), abs=1e-6)


def test_look_at_straight_down_is_stable():
    """★ 正下方是 look-at 的退化情況（視線與上方向平行），
    不處理會得到 NaN 或隨機滾轉。"""
    from pxr import Gf
    r = look_at_rotation((0.0, 0.0, 10.0), (0.0, 0.0, 0.0))
    assert all(math.isfinite(v) for v in r[0]) and math.isfinite(r[1])
    d = Gf.Rotation(*r).TransformDir(Gf.Vec3d(0, 0, -1))
    assert Gf.Vec3d(d)[2] == pytest.approx(-1.0, abs=1e-6)


# ── 相機避牆（2026-09-24 使用者：斜前方轉角穿牆、畫面全黑或全白）──────────

def test_oblique_and_chase_avoid_walls_topdown_does_not():
    from sim_cameras import CAMERAS
    by = {c.name: c for c in CAMERAS}
    assert by["oblique"].avoid_walls and by["chase"].avoid_walls
    assert not by["topdown"].avoid_walls        # 俯視在 5 m 高，這棟樓沒有天花板


def test_pull_stops_short_of_the_wall():
    from sim_cameras import WALL_MARGIN_M, pull_fraction
    assert pull_fraction(3.5, None) == 1.0
    f = pull_fraction(3.5, 2.0)
    assert abs(f * 3.5 - (2.0 - WALL_MARGIN_M)) < 1e-9


def test_pull_never_collapses_onto_the_car():
    from sim_cameras import MIN_PULL_FRACTION, pull_fraction
    assert pull_fraction(3.5, 0.1) == MIN_PULL_FRACTION


def test_pull_in_is_immediate_release_is_gradual():
    """★ 拉近要當幀生效（否則那一幀就在牆裡）；放遠要慢，不然畫面抽動。"""
    from sim_cameras import RELEASE_PER_FRAME, smooth_pull
    assert smooth_pull(1.0, 0.4) == 0.4
    assert abs(smooth_pull(0.4, 1.0) - (0.4 + RELEASE_PER_FRAME)) < 1e-12


def test_wall_ray_ends_at_the_nominal_eye():
    import math
    from sim_cameras import CAMERAS, camera_pose, pulled_eye, wall_ray
    cam = [c for c in CAMERAS if c.name == "oblique"][0]
    o, d, L = wall_ray(cam, (1.0, 2.0), 0.7, 0.1)
    eye, _ = camera_pose(cam, (1.0, 2.0), 0.7, 0.1)
    assert all(abs(a - b) < 1e-9 for a, b in zip(pulled_eye(o, d, L, 1.0), eye))
    assert abs(o[2] - eye[2]) < 1e-9            # 同高：水平射線，不會打到地板
    assert abs(math.hypot(*d) - 1.0) < 1e-9


def test_chase_checks_both_sides_oblique_does_not():
    """★ 2026-09-24：車後鏡頭轉角時牆角佔掉畫面一側 —— 兩側也要檢查。"""
    from sim_cameras import CAMERAS, side_rays
    by = {c.name: c for c in CAMERAS}
    rays = side_rays(by["chase"], (0.0, 0.0), 0.0, 0.0)
    assert len(rays) == 2
    ends = [(o[1] + d[1] * L) for o, d, L in rays]          # y 座標
    assert abs(abs(ends[0]) - by["chase"].side_clearance_m) < 1e-6
    assert ends[0] * ends[1] < 0                             # 一左一右
    assert side_rays(by["oblique"], (0.0, 0.0), 0.0, 0.0) == []
