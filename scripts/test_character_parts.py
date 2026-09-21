"""逐部位碰撞體規劃的測試（純計算，不需要 Isaac）。"""

from __future__ import annotations

import pytest

from character_parts import MIN_FACES_PER_PART, plan_parts, representative_joint

JOINTS = [
    "RL_BoneRoot",                                      # 0 torso
    "RL_BoneRoot/Hip/Pelvis",                           # 1 torso
    "RL_BoneRoot/Hip/Pelvis/L_Thigh",                   # 2 L_thigh
    "RL_BoneRoot/Hip/Pelvis/L_Thigh/L_Calf",            # 3 L_calf
    "RL_BoneRoot/Hip/Waist/Spine02/L_Clavicle/L_Upperarm",          # 4 L_upperarm
    "RL_BoneRoot/Hip/Waist/Spine02/L_Clavicle/L_Upperarm/L_Forearm",# 5 L_forearm
]


def test_representative_joint_is_the_one_closest_to_the_torso():
    """★ 骨架由根往外排，取第一個命中者 = 該部位最靠軀幹的骨頭，
    部位才會繞著解剖上正確的關節轉。"""
    assert representative_joint(JOINTS, "L_thigh") == 2
    assert representative_joint(JOINTS, "L_upperarm") == 4
    assert representative_joint(JOINTS, "torso") == 0


def test_absent_segment_has_no_representative():
    assert representative_joint(JOINTS, "R_calf") is None


def _grid(n_faces, seg_joint):
    """造 n_faces 個三角形，全部綁在 seg_joint 上。"""
    pts = [(float(i), 0.0, 0.0) for i in range(n_faces * 3)]
    counts = [3] * n_faces
    idx = list(range(n_faces * 3))
    ji = [seg_joint] * (n_faces * 3)
    jw = [1.0] * (n_faces * 3)
    return pts, counts, idx, ji, jw


def test_parts_are_bound_to_their_joint():
    pts, counts, idx, ji, jw = _grid(20, 2)          # 全綁 L_Thigh
    plan = plan_parts(JOINTS, pts, counts, idx, ji, jw, 1)
    assert "L_thigh" in plan
    assert plan["L_thigh"][3] == 2                    # joint index
    assert len(plan["L_thigh"][1]) == 20              # 20 個面


def test_tiny_fragments_are_dropped():
    """★ 太碎的塊 cook 失敗率高，且光達在 VLP-16 解析度下也分不出來。"""
    pts, counts, idx, ji, jw = _grid(MIN_FACES_PER_PART - 1, 2)
    assert "L_thigh" not in plan_parts(JOINTS, pts, counts, idx, ji, jw, 1)


def test_parts_do_not_share_faces():
    """★ 同一個面被兩塊覆蓋，光達會在接縫處量到兩層回波。"""
    n = 30
    pts = [(float(i), 0.0, 0.0) for i in range(n * 3)]
    counts, idx = [3] * n, list(range(n * 3))
    # 前 15 面綁大腿、後 15 面綁上臂
    ji = [2] * 45 + [4] * 45
    jw = [1.0] * (n * 3)
    plan = plan_parts(JOINTS, pts, counts, idx, ji, jw, 1)
    total = sum(len(v[1]) for v in plan.values())
    assert total == n, f"面數 {total} != {n}，有重複或遺漏"


def test_every_face_lands_somewhere():
    """沒有面可以憑空消失 —— 漏掉就是人身上破洞。"""
    n = 24
    pts = [(float(i), 0.0, 0.0) for i in range(n * 3)]
    counts, idx = [3] * n, list(range(n * 3))
    ji, jw = [1] * (n * 3), [1.0] * (n * 3)          # 全 torso
    plan = plan_parts(JOINTS, pts, counts, idx, ji, jw, 1)
    assert sum(len(v[1]) for v in plan.values()) == n
