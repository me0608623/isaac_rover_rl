"""移動障礙物運動模型與光達可見度的測試。

驗的是**物理量**：
  - 行人速度 1.2 m/s（成人平均步行），VLP-16 10 Hz → 每次掃描位移 12 cm
  - policy 的可見帶由 z_filter 決定，推車高度不足會完全看不到（實際踩過的 bug）
"""

from __future__ import annotations

import math

import pytest

from obstacle_motion import lidar_visible_height, path_length, position_at

WALK_SPEED = 1.2        # 成人平均步行速度 m/s
SENSOR_H = 1.43         # base_footprint → velodyne（0.13 + 1.30）
Z_FILTER = 0.5          # lidar_preprocessor_params_sa4r2.yaml


# ---------------------------------------------------------------- 可見度
def test_person_cylinder_is_visible_in_band():
    """1.70 m 人體代理：可見帶 [0.93, 1.93]，應露出 1.70 - 0.93 = 0.77 m。"""
    assert lidar_visible_height(1.70, SENSOR_H, Z_FILTER) == pytest.approx(0.77)


def test_low_cart_is_completely_invisible_to_policy():
    """★ 實際踩過的 bug：0.90 m 推車整個在可見帶下緣以下，policy 看不到。

    它仍有碰撞體（車撞得到），所以這是「撞得到但看不到」的危險組合。
    """
    assert lidar_visible_height(0.90, SENSOR_H, Z_FILTER) == 0.0


def test_cart_at_one_meter_only_shows_a_sliver():
    assert lidar_visible_height(1.00, SENSOR_H, Z_FILTER) == pytest.approx(0.07)


def test_minimum_height_to_be_seen_is_the_band_bottom():
    """要被看到，高度必須超過 sensor_h - z_filter。"""
    band_bottom = SENSOR_H - Z_FILTER
    assert lidar_visible_height(band_bottom, SENSOR_H, Z_FILTER) == 0.0
    assert lidar_visible_height(band_bottom + 0.01, SENSOR_H, Z_FILTER) > 0.0


def test_visible_height_saturates_at_band_top():
    """超高的障礙物也只露出一個帶寬（2 * z_filter）。"""
    assert lidar_visible_height(10.0, SENSOR_H, Z_FILTER) == pytest.approx(2 * Z_FILTER)


def test_raised_base_shifts_the_window():
    """放在 1.0 m 高台上的 0.5 m 箱子 → 佔 [1.0, 1.5]，全在帶內。"""
    assert lidar_visible_height(0.5, SENSOR_H, Z_FILTER, base_z=1.0) == pytest.approx(0.5)


# ---------------------------------------------------------------- 路徑長
def test_path_length_of_straight_line():
    assert path_length([(0.0, 0.0), (3.0, 4.0)]) == pytest.approx(5.0)


def test_path_length_accumulates_segments():
    assert path_length([(0.0, 0.0), (1.0, 0.0), (1.0, 2.0)]) == pytest.approx(3.0)


def test_single_waypoint_has_zero_length():
    assert path_length([(1.0, 2.0)]) == 0.0


# ---------------------------------------------------------------- 等速運動
def test_walks_at_the_commanded_speed():
    """1.2 m/s 走 1 s 應位移 1.2 m。"""
    wps = [(0.0, 0.0), (100.0, 0.0)]
    x, y, _ = position_at(wps, WALK_SPEED, 1.0)
    assert x == pytest.approx(WALK_SPEED)
    assert y == pytest.approx(0.0)


def test_displacement_per_lidar_sweep_is_12cm():
    """VLP-16 10 Hz：每次掃描間隔行人走 12 cm。"""
    wps = [(0.0, 0.0), (100.0, 0.0)]
    x0, _, _ = position_at(wps, WALK_SPEED, 0.0)
    x1, _, _ = position_at(wps, WALK_SPEED, 0.1)
    assert x1 - x0 == pytest.approx(0.12)


def test_turns_the_corner_of_a_polyline():
    """走完第一段 1 m 後再走 0.5 m，應在第二段上。"""
    wps = [(0.0, 0.0), (1.0, 0.0), (1.0, 5.0)]
    x, y, yaw = position_at(wps, 1.0, 1.5)
    assert (x, y) == pytest.approx((1.0, 0.5))
    assert yaw == pytest.approx(math.pi / 2)


def test_yaw_points_along_travel_direction():
    wps = [(0.0, 0.0), (0.0, 10.0)]
    _, _, yaw = position_at(wps, 1.0, 1.0)
    assert yaw == pytest.approx(math.pi / 2)


# ---------------------------------------------------------------- 邊界模式
def test_pingpong_reverses_at_the_end():
    """長 2 m 的路徑，1 m/s 走 3 s → 折返後回到 1 m 處。"""
    wps = [(0.0, 0.0), (2.0, 0.0)]
    x, _, yaw = position_at(wps, 1.0, 3.0, mode="pingpong")
    assert x == pytest.approx(1.0)
    assert yaw == pytest.approx(math.pi)      # 反向走


def test_pingpong_is_periodic():
    wps = [(0.0, 0.0), (2.0, 0.0)]
    a = position_at(wps, 1.0, 0.7, mode="pingpong")
    b = position_at(wps, 1.0, 0.7 + 4.0, mode="pingpong")   # 週期 = 2L/v = 4 s
    assert a == pytest.approx(b)


def test_loop_wraps_to_the_start():
    wps = [(0.0, 0.0), (2.0, 0.0)]
    x, _, _ = position_at(wps, 1.0, 2.5, mode="loop")
    assert x == pytest.approx(0.5)


def test_once_clamps_at_the_final_waypoint():
    wps = [(0.0, 0.0), (2.0, 0.0)]
    x, _, _ = position_at(wps, 1.0, 99.0, mode="once")
    assert x == pytest.approx(2.0)


def test_zero_speed_stays_put():
    wps = [(1.0, 2.0), (5.0, 2.0)]
    assert position_at(wps, 0.0, 50.0)[:2] == pytest.approx((1.0, 2.0))


def test_single_waypoint_never_moves():
    assert position_at([(3.0, 4.0)], 1.0, 10.0)[:2] == pytest.approx((3.0, 4.0))


def test_rejects_empty_waypoints():
    with pytest.raises(ValueError):
        position_at([], 1.0, 0.0)


# ------------------------------------------- 障礙物必須看得見
def test_every_default_obstacle_is_visible_to_the_policy():
    """★ 所有障礙物都必須進得了 policy 的可見帶，否則等於保證撞車。

    2026-09-21 實測：高 1.00 m 的 cart_b 只露出 7 cm，車全速撞上卡死
    （命令 |v|=0.577 但位移 0.000 m），導航失敗。把「撞得到但看不到」
    的物體放在必經路線上，測不到真正想測的動態避障。

    ⚠ 盲區本身在實車上是真的（同樣的光達高度與 z_filter=0.5），
    只是不該用它當一般導航測試的場景。
    """
    import ros_graph_spec as S
    for o in S.DEFAULT_OBSTACLES:
        v = lidar_visible_height(o.height, SENSOR_H, Z_FILTER)
        assert v > 0.3, f"{o.name} 高度 {o.height} m 只露出 {v:.2f} m，policy 幾乎看不到"


def test_obstacles_are_at_least_human_height():
    """障礙物高度一律 >= 1.5 m（與真人相當）。"""
    import ros_graph_spec as S
    for o in S.DEFAULT_OBSTACLES:
        assert o.height >= 1.5, f"{o.name} 高度 {o.height} m < 1.5 m"


def test_the_blind_spot_calculation_still_works():
    """盲區的計算本身保留 —— 要專門驗證矮障礙物風險時會用到。"""
    assert lidar_visible_height(0.90, SENSOR_H, Z_FILTER) == 0.0
    assert lidar_visible_height(1.00, SENSOR_H, Z_FILTER) == pytest.approx(0.07)


def test_obstacle_set_is_complete():
    """★ 守住「障礙物憑空消失」—— 2026-09-21 一次字串替換把 cart_a 整行吃掉，
    而當時沒有任何測試擋得住：其他測試都是「對每個障礙物檢查…」，
    少一個反而更容易通過。
    """
    import ros_graph_spec as S
    names = {o.name for o in S.DEFAULT_OBSTACLES}
    assert names == {"person_a", "person_b", "person_c", "cart_a", "cart_b"}, \
        f"障礙物組合不對：{sorted(names)}"
