"""骨骼→身體部位分群的測試。

分群錯了會讓碰撞體掛到錯的骨頭上，手臂動的時候大腿跟著飛 —— 這種錯在
點雲上很難察覺，所以要逐條驗。
"""

from __future__ import annotations

import pytest

from skel_parts import (SEGMENTS, assign_faces, assign_vertices, joint_to_segment)

# 取自 F_Business_02 的 Reallusion 骨架（101 關節）
J = [
    "RL_BoneRoot",
    "RL_BoneRoot/Hip",
    "RL_BoneRoot/Hip/Pelvis",
    "RL_BoneRoot/Hip/Pelvis/L_Thigh",
    "RL_BoneRoot/Hip/Pelvis/L_Thigh/L_Calf",
    "RL_BoneRoot/Hip/Pelvis/L_Thigh/L_Calf/L_Foot",
    "RL_BoneRoot/Hip/Pelvis/R_Thigh",
    "RL_BoneRoot/Hip/Waist/Spine01/Spine02/L_Clavicle/L_Upperarm",
    "RL_BoneRoot/Hip/Waist/Spine01/Spine02/L_Clavicle/L_Upperarm/L_Forearm",
    "RL_BoneRoot/Hip/Waist/Spine01/Spine02/L_Clavicle/L_Upperarm/L_Forearm/L_Hand",
    "RL_BoneRoot/Hip/Waist/Spine01/Spine02/NeckTwist01/Head",
]


def test_limb_joints_map_to_their_own_segment():
    assert joint_to_segment(J[3]) == "L_thigh"
    assert joint_to_segment(J[4]) == "L_calf"
    assert joint_to_segment(J[6]) == "R_thigh"
    assert joint_to_segment(J[7]) == "L_upperarm"
    assert joint_to_segment(J[8]) == "L_forearm"


def test_foot_and_hand_follow_their_parent_limb():
    """腳掌併入小腿、手掌併入前臂 —— 單獨成塊太小，點雲上分不出來。"""
    assert joint_to_segment(J[5]) == "L_calf"
    assert joint_to_segment(J[9]) == "L_forearm"


def test_head_and_neck_are_one_segment():
    assert joint_to_segment(J[10]) == "head"
    assert joint_to_segment("RL_BoneRoot/Hip/Waist/Spine01/NeckTwist01") == "head"


def test_spine_and_pelvis_are_torso():
    for j in (J[1], J[2], "RL_BoneRoot/Hip/Waist/Spine01/Spine02"):
        assert joint_to_segment(j) == "torso"


def test_clavicle_belongs_to_torso_not_the_arm():
    """鎖骨幾乎不動，歸軀幹可避免肩部在兩塊碰撞體間裂開。"""
    assert joint_to_segment("RL_BoneRoot/Hip/Waist/Spine01/Spine02/L_Clavicle") == "torso"


def test_root_falls_back_to_torso():
    assert joint_to_segment("RL_BoneRoot") == "torso"


def test_left_right_are_never_confused():
    """★ L/R 搞反會讓左手的碰撞體跟著右手跑。"""
    for seg in SEGMENTS:
        if seg.startswith("L_"):
            assert seg.replace("L_", "R_", 1) in SEGMENTS


def test_vertex_takes_its_dominant_bone():
    """每個頂點有多個權重，取最大的那個決定歸屬。"""
    joints = [J[3], J[7]]                 # [L_thigh, L_upperarm]
    idx = [0, 1, 0, 1]                    # 2 頂點 × 2 骨
    w = [0.9, 0.1, 0.2, 0.8]
    assert assign_vertices(idx, w, 2, joints) == ["L_thigh", "L_upperarm"]


def test_zero_weight_vertex_falls_back_to_torso():
    joints = [J[3], J[7]]
    assert assign_vertices([0, 1], [0.0, 0.0], 2, joints) == ["torso"]


def test_face_goes_to_the_majority_segment():
    """跨部位的三角形整片歸多數方，避免同一面被兩塊碰撞體重複覆蓋。"""
    vseg = ["torso", "torso", "L_upperarm"]
    assert assign_faces([3], [0, 1, 2], vseg) == ["torso"]


def test_face_tie_is_resolved_deterministically():
    """平手時必須有確定結果，否則每次建出來的碰撞體都不一樣。"""
    vseg = ["torso", "L_upperarm", "L_upperarm", "torso"]
    a = assign_faces([4], [0, 1, 2, 3], vseg)
    b = assign_faces([4], [0, 1, 2, 3], vseg)
    assert a == b and a[0] in ("torso", "L_upperarm")


def test_handles_mixed_polygon_sizes():
    vseg = ["head"] * 7
    assert assign_faces([3, 4], [0, 1, 2, 3, 4, 5, 6], vseg) == ["head", "head"]
