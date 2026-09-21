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


def character_pose_at(walk, sim_time: float, floor_top: float):
    """回傳角色在時刻 ``sim_time`` 的 ``(world_x, world_y, world_z, world_yaw)``。

    Args:
        walk:      一個 :class:`ros_graph_spec.CharacterWalk`。
        sim_time:  模擬時間 s。
        floor_top: 走廊地板頂面的 world z。角色原點在腳底，所以 z 就是它。
    """
    if not walk.waypoints:
        return (0.0, 0.0, floor_top, 0.0)
    mx, my, myaw = position_at(walk.waypoints, walk.speed,
                              sim_time + walk.phase_s, walk.mode)
    wx, wy, wyaw = S.map_to_world(mx, my, myaw)
    return (wx, wy, floor_top, wyaw)
