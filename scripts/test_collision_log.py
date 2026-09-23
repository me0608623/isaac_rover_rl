"""擦撞紀錄的測試（不需要 Isaac）。"""

from __future__ import annotations

import math

import pytest

from collision_log import (BODY_BOX_CENTRE, BODY_BOX_HALF, FLOOR_LIFT_M,
                           EpisodeTracker, body_box_world, classify_hit,
                           is_ground_contact, is_self, summarise)

R = "/World/charger_rover4_5_0/charger_rover_urdf5"


def test_body_box_covers_the_measured_visual_body():
    """外觀車身實測（base_link 座標）：x −0.458~+0.212、y ±0.277、頂 +1.575。"""
    cx, cy, cz = BODY_BOX_CENTRE
    hx, hy, hz = BODY_BOX_HALF
    assert cx - hx == pytest.approx(-0.458, abs=1e-3)
    assert cx + hx == pytest.approx(0.212, abs=1e-3)
    assert hy == pytest.approx(0.277, abs=1e-3)
    assert cz + hz == pytest.approx(1.575, abs=1e-3)


def test_body_box_bottom_is_lifted_off_the_floor():
    """★★ 盒子底部貼地的話每一步都會「碰到地板」，而地板跟牆是同一個
    Mesh_015 —— 整趟都會被記成撞牆。地板在 base_link z = −0.134。"""
    floor = -0.134
    bottom = BODY_BOX_CENTRE[2] - BODY_BOX_HALF[2]
    assert bottom - floor == pytest.approx(FLOOR_LIFT_M, abs=1e-3)


def test_body_box_is_much_wider_than_the_physics_chassis():
    """★★ 物理引擎的底盤碰撞體只是 0.17 × 0.47 m 的薄板，外觀是 0.67 × 0.55 m。
    只靠物理碰撞回報會漏掉「外殼擦過、薄板沒碰到」的擦撞。"""
    assert 2 * BODY_BOX_HALF[1] > 0.17 * 3
    assert 2 * BODY_BOX_HALF[0] > 0.47


def test_classify_hit():
    walking = frozenset({"Character_10"})
    assert classify_hit("/World/SimObstacles/prop_4_0/Collider", walking) == ("道具", "prop_4_0")
    assert classify_hit("/World/SimObstacles/pair_4_1", walking) == ("人形圓柱", "pair_4_1")
    assert classify_hit("/World/SimObstacles/ped_2_3", walking) == ("人形圓柱", "ped_2_3")
    assert classify_hit("/World/Characters/Character_10/parts/l_arm", walking) == ("走動行人", "Character_10")
    assert classify_hit("/World/Characters/Character_17/parts/torso", walking) == ("站立行人", "Character_17")
    assert classify_hit("/World/Env_0/Floor/_F/Mesh_015", walking) == ("牆", "Mesh_015")
    assert classify_hit("/World/NavFloor_H", walking) == ("地板", "NavFloor_H")


def test_self_hits_are_not_collisions():
    """★ 盒子一定包住車自己的碰撞體 —— 不排除的話每一步都是擦撞。"""
    assert is_self(f"{R}/base_link/collisions/mesh_0")
    assert classify_hit(f"{R}/left_wheel/collisions/mesh_0") is None


def test_ground_contact_is_recognised_only_for_wheels_pushing_down():
    """★ 輪子貼地是正常的（當心跳用）；但底盤撞到牆（法向量水平）是真的擦撞。"""
    assert is_ground_contact(f"{R}/left_wheel/collisions/mesh_0", "地板", 0.98)
    assert is_ground_contact(f"{R}/right_caster_wheel_link/c", "牆", -0.95)
    assert not is_ground_contact(f"{R}/base_link/collisions/mesh_0", "牆", 0.1)
    assert not is_ground_contact(f"{R}/left_wheel/collisions/mesh_0", "牆", 0.1)


def test_body_box_follows_the_robot_pose():
    """盒子中心要跟著車轉：車頭朝 +y（轉 90°）時，偏後 0.123 m 的中心要落在 −y。"""
    from pxr import Gf

    m = Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90.0)) * \
        Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(1.0, 2.0, 0.0))
    (cx, cy, cz), (qx, qy, qz, qw) = body_box_world(m)
    assert (cx, cy) == pytest.approx((1.0, 2.0 - 0.123), abs=1e-6)
    assert cz == pytest.approx(BODY_BOX_CENTRE[2], abs=1e-6)
    assert qw == pytest.approx(math.cos(math.pi / 4), abs=1e-6)
    assert qz == pytest.approx(math.sin(math.pi / 4), abs=1e-6)


def test_episode_tracker_merges_continuous_frames():
    tr = EpisodeTracker(gap_s=0.2)
    for i in range(10):
        tr.hit(1.0 + i / 60, ("道具", "prop_4_0"))
    tr.hit(3.0, ("道具", "prop_4_0"))              # 隔了快 2 秒 → 另一次
    ep = tr.close()
    assert len(ep) == 2
    assert ep[0][2] == pytest.approx(1.0) and ep[0][3] == pytest.approx(1.0 + 9 / 60)


def test_summary_flags_a_detector_that_never_saw_itself():
    """★★ 偵測器壞掉時最危險的輸出是「0 次擦撞」。重疊查詢每一步都該看到
    車自己；看不到就標成沒在工作，而不是報 0。"""
    s = summarise([], overlap_steps=1000, overlap_self_steps=0, ground_contacts=0)
    assert s["episodes"] == 0
    assert s["detector"]["overlap_ok"] is False
    assert s["detector"]["contact_report_ok"] is False


def test_summary_trusts_a_detector_that_saw_itself_every_step():
    ep = [("道具", "prop_4_0", 1.0, 1.5), ("走動行人", "Character_10", 5.0, 5.1)]
    s = summarise(ep, overlap_steps=1000, overlap_self_steps=1000, ground_contacts=4200)
    assert s["detector"]["overlap_ok"] and s["detector"]["contact_report_ok"]
    assert s["by_category"]["道具"] == {"episodes": 1, "seconds": 0.5}
    assert s["episodes"] == 2


def test_zero_steps_is_not_ok():
    """一步都沒跑（例如沒接上主迴圈）不能算成「看到自己 100%」。"""
    assert summarise([], 0, 0, 10)["detector"]["overlap_ok"] is False


def test_snapshot_includes_open_episodes_without_closing_them():
    """★ 定期存檔要包含「還在擦」的那一次，但不能把它關掉 —— 關掉的話
    下一幀同一次擦撞會被拆成兩次。"""
    tr = EpisodeTracker(gap_s=0.2)
    tr.hit(1.0, ("道具", "p"))
    tr.hit(1.1, ("道具", "p"))
    snap = tr.snapshot()
    assert snap == [("道具", "p", 1.0, 1.1)]
    tr.hit(1.2, ("道具", "p"))                    # 仍是同一次
    assert tr.close() == [("道具", "p", 1.0, 1.2)]


def test_flush_interval_is_short_enough_to_survive_a_kill():
    """★★ 批次收尾是 SIGINT、6 秒後 kill -9，Isaac 的 finally 從來沒跑過。
    摘要必須定期寫，間隔要遠小於那 6 秒。"""
    from collision_log import FLUSH_EVERY_S
    assert FLUSH_EVERY_S <= 2.0


def test_one_person_touch_is_one_event_not_two():
    """★★ 站立行人腳邊疊了隱形碰撞圓柱，碰到那個人時兩個都會被回報。
    總數要是 1 次事件，不是 2 次。"""
    ep = [("人形圓柱", "pair_4_1", 3.00, 3.40),
          ("站立行人", "Character_17", 3.02, 3.38)]
    s = summarise(ep, 1000, 1000, 10)
    assert s["episodes"] == 1
    assert s["by_category"]["人形圓柱"]["episodes"] == 1
    assert s["by_category"]["站立行人"]["episodes"] == 1


def test_separate_touches_stay_separate():
    ep = [("道具", "prop_4_0", 1.0, 1.2), ("走動行人", "Character_10", 5.0, 5.1)]
    assert summarise(ep, 1000, 1000, 10)["episodes"] == 2
