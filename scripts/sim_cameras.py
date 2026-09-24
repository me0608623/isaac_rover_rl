"""論文錄影用的三視角相機（純幾何，不依賴 Isaac）。

不用 GUI 螢幕錄影的理由：GUI 要處理視窗管理、解析度、遮擋與錄影軟體，
而且一次只能拍一個視角。用 stage 裡的相機 + render product 可以**三個
視角同時算**，畫面乾淨、框取精確、完全自動化，也不依賴 X11。

三個視角：
  topdown  俯視 —— 位置跟著車走，**方向固定朝世界正下方**
  chase    車後 —— 在車後上方，跟著車頭轉，看向行進方向
  oblique  斜前方旁觀 —— 在車前方偏側，回頭看車

⚠ 2026-09-22 的兩個幾何誤判，都是拿**錯的量測**當定論，記在這裡免得再犯：
  1.「走廊天花板在離地 3.65 m」—— 那是整棟樓 Mesh_015（一顆 Mesh 包了
     全樓層）bbox 的 z 最大值，是全樓最高點，不是走廊局部高度。
     實際把相機朝正上方拍，畫面是**均勻的天空背景**：這棟樓根本**沒有天花板**。
     所以俯視相機不必受 3.65 m 限制，也沒有「天花板遮住視線」的問題。
  2.「斜前方 3.5 m」—— 實拍發現相機在**牆裡面**，畫面六成是白牆。
     走廊沒那麼寬，側向位移要收在 CORRIDOR_HALF_WIDTH_M 以內。
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class CameraSpec:
    """一個視角的設定。

    offset 是在**車體座標系**下的位移（x 前、y 左、z 上），
    再依 ``follow_yaw`` 決定要不要跟著車頭旋轉。
    """

    name: str
    offset: tuple[float, float, float]
    #: True = offset 隨車頭旋轉（車後視角需要）；False = 世界軸固定
    follow_yaw: bool
    #: 相機看向的目標在車體座標系的位移。None = 看車本身（原點）
    target_offset: tuple[float, float, float] = (0.0, 0.0, 0.5)
    #: 目標是否也隨車頭旋轉
    target_follow_yaw: bool = True
    #: 視野角（度）
    focal_length_mm: float = 18.0
    #: True = 相機會被牆擋時往車子方向拉近（見 pulled_eye）
    avoid_walls: bool = False
    #: 鏡頭可往走廊中間橫移的最大距離（m）；0 = 不橫移。
    #: 2026-09-24：車貼牆走時，車後鏡頭也貼著同一面牆，牆佔掉畫面一側。
    #: 試過「兩側射線」與「周圍淨空」都無效（車本身就在牆邊，拉近沒用），
    #: 改成量鏡頭左右離牆距離、往寬的一側移（每幀最多 2 cm，平順）。
    side_clearance_m: float = 0.0


#: USD 相機的預設光圈（mm）。FOV = 2*atan(aperture / (2*focal))。
#: 焦距換算涵蓋範圍時要用它，不能憑感覺猜。
HORIZONTAL_APERTURE_MM = 20.955
VERTICAL_APERTURE_MM = 15.2908

#: 走廊半寬（m，保守值）。相機的側向位移超過這個值就會埋進牆裡 ——
#: 2026-09-22 斜前方相機設 3.5 m，實拍畫面六成是白牆。
CORRIDOR_HALF_WIDTH_M = 1.6


CAMERAS: tuple[CameraSpec, ...] = (
    # 俯視：正上方 5.0 m，焦距 12 mm → 地面涵蓋約 8.8 m 寬。
    # ⚠ follow_yaw=False —— 跟著車轉的話畫面會一直旋轉，看的人會暈；
    #   位置跟著走、方向固定朝下才好判讀走位。
    # 高度 5 m 越過所有角色（最高 2.1 m 離地）與門框，且這棟樓沒有天花板。
    CameraSpec("topdown", (0.0, 0.0, 5.0), follow_yaw=False,
               target_offset=(0.0, 0.0, 0.0), target_follow_yaw=False,
               focal_length_mm=12.0),
    # 車後：後方 2.8 m、高 2.0 m，跟著車頭轉，看向前方 3 m 處。
    # 高度刻意高過行人頭頂（1.8 m），起點人群密集時才不會整台車被擋住。
    CameraSpec("chase", (-2.8, 0.0, 2.0), follow_yaw=True,
               target_offset=(3.0, 0.0, 0.3), target_follow_yaw=True,
               focal_length_mm=20.0, avoid_walls=True, side_clearance_m=0.8),
    # 斜前方旁觀：前方 3.2 m、左 1.4 m、高 1.7 m，回頭看車。
    # 側向 1.4 m 是貼著 CORRIDOR_HALF_WIDTH_M 的上限走，再多就進牆。
    CameraSpec("oblique", (3.2, 1.4, 1.7), follow_yaw=True,
               target_offset=(0.0, 0.0, 0.5), target_follow_yaw=True,
               focal_length_mm=24.0, avoid_walls=True),
)


def ground_coverage_m(cam: CameraSpec, height_m: float | None = None):
    """回傳這台相機在地面上涵蓋的 ``(寬, 高)``（m）。

    只對**垂直朝下**的相機有意義（斜視角的涵蓋範圍是梯形）。
    用來檢查俯視圖的框取：涵蓋太窄看不到周遭，太寬車就變成幾個像素。
    """
    h = cam.offset[2] if height_m is None else height_m
    w = 2.0 * h * (HORIZONTAL_APERTURE_MM / (2.0 * cam.focal_length_mm))
    v = 2.0 * h * (VERTICAL_APERTURE_MM / (2.0 * cam.focal_length_mm))
    return (w, v)


def _apply(offset, yaw: float, follow: bool):
    """把車體座標的位移轉到世界座標。"""
    ox, oy, oz = offset
    if not follow:
        return (ox, oy, oz)
    c, s = math.cos(yaw), math.sin(yaw)
    return (ox * c - oy * s, ox * s + oy * c, oz)


def look_at_rotation(eye, target, up=(0.0, 0.0, 1.0)):
    """回傳讓相機從 ``eye`` 看向 ``target`` 的旋轉 ``(axis, angle_deg)``。

    ⚠ USD 相機看向自身 **-Z** 軸（+Y 為上）。算成 +Z 會拍到反方向。

    ⚠ 正下方是退化情況（視線與 up 平行），不特判會得到 NaN 或隨機滾轉。
    """
    import numpy as np

    fwd = np.array(target, dtype=float) - np.array(eye, dtype=float)
    n = np.linalg.norm(fwd)
    if n < 1e-9:
        fwd = np.array([0.0, 0.0, -1.0])
    else:
        fwd = fwd / n
    u = np.array(up, dtype=float)
    if abs(float(np.dot(fwd, u))) > 0.999:        # 視線幾乎垂直 → 換參考上方向
        u = np.array([0.0, 1.0, 0.0])
    right = np.cross(fwd, u)
    right /= np.linalg.norm(right)
    true_up = np.cross(right, fwd)
    # 相機基底：X=right, Y=up, Z=-forward
    m = np.stack([right, true_up, -fwd], axis=1)
    # 旋轉矩陣 → 軸角
    tr = float(np.trace(m))
    ang = math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))
    if ang < 1e-9:
        return ((0.0, 0.0, 1.0), 0.0)
    if abs(ang - math.pi) < 1e-6:                 # 180° 退化
        d = np.diag(m)
        k = int(np.argmax(d))
        axis = np.zeros(3)
        axis[k] = math.sqrt(max(0.0, (d[k] + 1.0) / 2.0))
        return (tuple(axis / np.linalg.norm(axis)), 180.0)
    axis = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
    axis /= (2.0 * math.sin(ang))
    return (tuple(axis), math.degrees(ang))


def camera_pose(cam: CameraSpec, robot_xy, yaw: float, floor_z: float):
    """回傳相機的 ``((x, y, z), (axis, angle_deg))``（世界座標）。

    Args:
        robot_xy: 車在世界座標的 (x, y)
        yaw:      車頭方向（世界座標，rad）
        floor_z:  地板高度，相機高度以它為基準
    """
    ox, oy, oz = _apply(cam.offset, yaw, cam.follow_yaw)
    eye = (robot_xy[0] + ox, robot_xy[1] + oy, floor_z + oz)
    tx, ty, tz = _apply(cam.target_offset, yaw, cam.target_follow_yaw)
    target = (robot_xy[0] + tx, robot_xy[1] + ty, floor_z + tz)
    return eye, look_at_rotation(eye, target)


# ── 相機避牆（2026-09-24）──────────────────────────────────────────────
# ⚠ 使用者回報：斜前方視角在走廊轉角、盡頭與迴轉時「有幾秒穿進牆裡，
#   畫面全黑或全白」。固定的車體座標位移在直走廊沒問題，但車一靠近牆或
#   轉向牆，前方 3.2 m 就在牆裡。
#   作法：每幀從車（相機同高）朝預定相機位置打射線，被牆擋就把相機拉到
#   擋點前 WALL_MARGIN_M；拉近「立刻」、放遠「慢慢」，畫面才不會抽動。

#: 相機與牆面保持的距離（m）。相機近裁切面 0.05 m，留 0.3 m 才不會拍到牆的截面。
WALL_MARGIN_M = 0.3
#: 拉得再近也不小於這個比例 —— 貼到車身上就什麼都看不到了。
MIN_PULL_FRACTION = 0.15
#: 放遠時每幀最多恢復多少比例（30 fps 下約 1 秒回到原位）。
RELEASE_PER_FRAME = 0.035


def wall_ray(cam: CameraSpec, robot_xy, yaw: float, floor_z: float):
    """回傳 ``(origin, unit_dir, length)``：從車（相機高度）到預定相機位置的射線。"""
    eye, _ = camera_pose(cam, robot_xy, yaw, floor_z)
    origin = (robot_xy[0], robot_xy[1], eye[2])
    d = (eye[0] - origin[0], eye[1] - origin[1], eye[2] - origin[2])
    n = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
    if n < 1e-9:
        return origin, (1.0, 0.0, 0.0), 0.0
    return origin, (d[0] / n, d[1] / n, d[2] / n), n


def pull_fraction(length: float, hit_distance) -> float:
    """射線在 ``hit_distance`` 被擋 → 相機該停在全長的多少比例（0~1]。

    ``hit_distance`` 為 None（沒擋到）回 1.0。
    """
    if hit_distance is None or length <= 0:
        return 1.0
    f = (hit_distance - WALL_MARGIN_M) / length
    return max(MIN_PULL_FRACTION, min(1.0, f))


def smooth_pull(prev: float, target: float) -> float:
    """拉近立刻跟上（避免任何一幀進牆），放遠每幀最多恢復 RELEASE_PER_FRAME。"""
    if target <= prev:
        return target
    return min(target, prev + RELEASE_PER_FRAME)


def pulled_eye(origin, unit_dir, length: float, fraction: float):
    """沿射線取相機位置。"""
    k = length * fraction
    return (origin[0] + unit_dir[0] * k, origin[1] + unit_dir[1] * k,
            origin[2] + unit_dir[2] * k)

