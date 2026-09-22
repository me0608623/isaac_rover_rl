"""角色沿路徑行走的位姿計算（純函數，不依賴 Isaac / USD）。

與已移除的圓柱 walker 的關鍵差異：

- **原點在腳底**。圓柱的原點在幾何中心，所以擺位要加半個身高；角色的
  骨架原點在腳底，z 直接取地板高度。加錯會讓人浮在半空或陷進地板。
- **朝向要轉**。角色的靜止朝向是 -Y（見 CHARACTER_REST_FACING_DEG），
  沿路徑走時必須繞 Z 轉到行進方向，否則會倒著走。
"""

from __future__ import annotations

import math

import ros_graph_spec as S
from obstacle_motion import position_at


def facing_rotation_deg(target_yaw_rad: float,
                        rest_facing_deg: float = None) -> float:
    """要繞 Z 轉幾度，才能讓角色從靜止朝向轉到 ``target_yaw_rad``。"""
    if rest_facing_deg is None:
        rest_facing_deg = S.CHARACTER_REST_FACING_DEG
    return math.degrees(target_yaw_rad) - rest_facing_deg


def foot_offset_from_bbox(origin_z: float, bbox_min_z: float) -> float:
    """角色腳底在 prim 原點**下方**多少（m）。

    ⚠ 別假設原點就在腳底。2026-09-22 實測 20 個 People 角色，
    原點都在腳底上方 0.119~0.151 m，而且每個角色不一樣。
    """
    return origin_z - bbox_min_z


def origin_z_for_feet_on_floor(floor_top: float, foot_offset: float) -> float:
    """要讓腳底剛好踩在 ``floor_top``，prim 原點的 z 該放哪。"""
    return floor_top + foot_offset


def character_pose_at(walk, sim_time: float, floor_top: float,
                      foot_offset: float = 0.0):
    """回傳角色在時刻 ``sim_time`` 的 ``(world_x, world_y, world_z, world_yaw)``。

    Args:
        walk:      一個 :class:`ros_graph_spec.CharacterWalk`。
        sim_time:  模擬時間 s。
        floor_top:   走廊地板頂面的 world z。
        foot_offset: 腳底在原點下方多少（見 foot_offset_from_bbox）。
                     ⚠ 預設 0 只為相容舊呼叫；實際使用一定要給，
                     不然腳會陷進地板 12~15 cm。
    """
    if not walk.waypoints:
        return (0.0, 0.0, origin_z_for_feet_on_floor(floor_top, foot_offset), 0.0)
    mx, my, myaw = position_at(walk.waypoints, walk.speed,
                              sim_time + walk.phase_s, walk.mode)
    wx, wy, wyaw = S.map_to_world(mx, my, myaw)
    return (wx, wy, origin_z_for_feet_on_floor(floor_top, foot_offset), wyaw)
