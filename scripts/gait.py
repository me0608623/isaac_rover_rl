"""程序化步態：直接在 Reallusion 骨架上產生走路姿勢（純函數）。

為什麼不用 NVIDIA 的動捕片段：
`People/Animations/stand_walk_*.skelanim.usd` 用的是通用 biped 命名
（Root / Pelvis / R_UpLeg / R_LoLeg / R_Ankle），而 People 角色的骨架是
Reallusion 命名（RL_BoneRoot / Hip / L_Thigh / L_Calf / L_Foot），81 vs 101
個關節。兩者之間要做綁定姿勢補償的 retargeting，那正是 omni.anim.people
的動畫圖在做的事 —— 而那條執行期在 headless 下起不來（實測角色根節點
180 步位移 0.000 m，get_character() 取不到）。

為什麼程序化就夠：需求是「讓碰撞體的四肢擺動，使光達掃出走路中的人形」。
VLP-16 在 5 m 處的角解析約 1.7 cm，而 policy 的 72-bin sweep 每 bin 5° ——
程序化步態與動捕步態在點雲上分不出來。換來的是**完全確定性**：同一個
模擬時間必得同一個姿勢，實驗可重現（行為腳本驅動的走路做不到這點）。

步態模型是正弦擺動，但受三個真人步態的約束（皆有測試把關）：
  - 左右腿反相，否則變成雙腳同時離地的兔子跳
  - 同側手腳反相，這是人類步態最顯著的特徵
  - 膝蓋只能單向彎，不能反折
"""

from __future__ import annotations

import math

#: 會擺動的部位（對應 skel_parts.SEGMENTS 中的四肢）。
GAIT_JOINTS: tuple[str, ...] = (
    "L_thigh", "R_thigh", "L_calf", "R_calf",
    "L_upperarm", "R_upperarm", "L_forearm", "R_forearm",
)

#: 常速步行（1.2 m/s）的步態週期 s。單腳來回一次約 1.1 s。
_BASE_PERIOD_S = 1.10
_BASE_SPEED = 1.2

#: 各關節在 1.2 m/s 時的擺幅（rad）。取自一般步態分析的量級：
#: 髖 ±25°、膝 0~35°（單向）、肩 ±20°、肘 0~25°（單向）。
_AMPLITUDE_RAD = {
    "L_thigh": 0.44, "R_thigh": 0.44,          # 髖屈伸 ±25°
    "L_calf": 0.31, "R_calf": 0.31,            # 膝彎曲 35°，單向
    "L_upperarm": 0.35, "R_upperarm": 0.35,    # 肩擺 ±20°
    "L_forearm": 0.22, "R_forearm": 0.22,      # 肘彎 25°，單向
}

#: 只能單向彎的關節（膝、肘）—— 反折在解剖上不可能。
_UNIDIRECTIONAL = ("L_calf", "R_calf", "L_forearm", "R_forearm")

#: 基礎姿勢修正（rad），與相位無關的靜態偏移。
#:
#: ⚠ 為什麼手臂需要：NVIDIA People 的綁定姿勢是 **T-pose**，手臂沿 ±X 平舉
#: （實測 1.27 m 高度處寬 1.56 m）。走路的人手臂垂在身側，所以要先繞前後軸
#: 把手臂放下來。這同時也是「光達打到的人張開雙臂」問題的解法。
#:
#: 取 78° 而非 90°：完全垂直會讓手臂緊貼軀幹而穿模，留一點外張比較自然。
#: 左右從 ±X 各自往下轉，所以符號相反。
BASE_POSE_RAD = {
    "L_upperarm": math.radians(78.0),
    "R_upperarm": math.radians(-78.0),
    "L_forearm": math.radians(10.0),      # 手肘微屈
    "R_forearm": math.radians(-10.0),
    "L_thigh": 0.0, "R_thigh": 0.0,       # 腿在 T-pose 已朝下，不需修正
    "L_calf": 0.0, "R_calf": 0.0,
}


def base_pose_angles() -> dict[str, float]:
    """與相位無關的基礎姿勢偏移（rad）。"""
    return dict(BASE_POSE_RAD)


def cycle_period(speed: float) -> float:
    """步態週期 s。走得快步頻高，週期用平方根縮放（步態研究的經驗關係）。"""
    if speed <= 0.0:
        return _BASE_PERIOD_S
    return _BASE_PERIOD_S * math.sqrt(_BASE_SPEED / speed)


def stride_phase(t: float, speed: float) -> float:
    """模擬時間 → 步態相位 [0, 2π)。"""
    return (2.0 * math.pi * t / cycle_period(speed)) % (2.0 * math.pi)


def joint_angles(phase: float, speed: float) -> dict[str, float]:
    """給定相位與速度，回傳各關節的擺動角（rad，繞左右軸）。

    速度為 0 時全部回 0 —— 站著不動的人不該有殘留擺動。
    """
    if speed <= 0.0:
        return {j: 0.0 for j in GAIT_JOINTS}

    # 擺幅隨速度線性成長，但設上限避免高速時誇張。
    scale = min(1.6, max(0.3, speed / _BASE_SPEED))
    out: dict[str, float] = {}
    for j in GAIT_JOINTS:
        amp = _AMPLITUDE_RAD[j] * scale
        # 右側落後半個週期 → 左右反相。
        p = phase + (math.pi if j.startswith("R_") else 0.0)
        # 手臂再偏移半週期 → 同側手腳反相。
        if "arm" in j:
            p += math.pi
        if j in _UNIDIRECTIONAL:
            # 膝、肘：用 (1 - cos)/2 ∈ [0,1]，恆為單向彎曲。
            out[j] = -amp * (1.0 - math.cos(2.0 * p)) * 0.5
        else:
            out[j] = amp * math.sin(p)
    return out
