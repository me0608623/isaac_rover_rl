"""把角色骨架與蒙皮網格拆成身體部位（純函數，不依賴 Isaac / USD）。

為什麼要拆：PhysX 的三角網格碰撞體是 cook 一次就固定，不會跟著骨架變形。
整具人體做成一塊碰撞體，只能停在綁定姿勢（而 NVIDIA People 的綁定姿勢是
T-pose，手臂平舉 1.56 m，比圓柱還不像行人）。

拆成部位之後，每塊各自 cook 並跟著對應骨骼的變換走 —— 四肢就會真的擺動，
而且每塊都保留真實輪廓，不是膠囊近似。每幀成本只是剛體變換。

分群原則：
  - 粒度取「光達分得出來」為準。VLP-16 在 5 m 處的角解析約 1.7 cm，
    手指、腳趾這種尺度沒有意義，所以手掌併入前臂、腳掌併入小腿。
  - 鎖骨歸軀幹：它幾乎不動，歸手臂會讓肩部在兩塊碰撞體之間裂開。
"""

from __future__ import annotations

from collections import Counter

#: 身體部位。順序固定，建出來的碰撞體才可重現。
SEGMENTS: tuple[str, ...] = (
    "head", "torso",
    "L_upperarm", "L_forearm", "R_upperarm", "R_forearm",
    "L_thigh", "L_calf", "R_thigh", "R_calf",
)

#: 骨骼葉名關鍵字 → 部位。由上而下第一個命中者勝，所以順序有意義：
#: 例如 "L_ForearmTwist" 必須在 "L_Forearm" 之前或用 startswith 才不會誤判。
_RULES: tuple[tuple[str, tuple[str, ...]], ...] = (
    ("head",       ("head", "neck", "eye", "jaw", "tongue", "teeth", "facial")),
    ("L_forearm",  ("l_forearm", "l_hand", "l_index", "l_mid", "l_ring",
                    "l_pinky", "l_thumb")),
    ("R_forearm",  ("r_forearm", "r_hand", "r_index", "r_mid", "r_ring",
                    "r_pinky", "r_thumb")),
    ("L_upperarm", ("l_upperarm", "l_elbow")),
    ("R_upperarm", ("r_upperarm", "r_elbow")),
    ("L_calf",     ("l_calf", "l_foot", "l_toe")),
    ("R_calf",     ("r_calf", "r_foot", "r_toe")),
    ("L_thigh",    ("l_thigh", "l_knee")),
    ("R_thigh",    ("r_thigh", "r_knee")),
    # 鎖骨、脊椎、骨盆都歸軀幹（見模組說明）
    ("torso",      ("clavicle", "spine", "pelvis", "hip", "waist", "chest",
                    "breast", "shoulder", "root")),
)

#: 對不上任何規則時的歸屬。寧可多算進軀幹，也不要漏掉頂點。
FALLBACK_SEGMENT = "torso"


def joint_to_segment(joint_path: str) -> str:
    """骨骼路徑（如 ``RL_BoneRoot/Hip/Pelvis/L_Thigh``）→ 部位名。"""
    leaf = joint_path.rsplit("/", 1)[-1].lower()
    for segment, keys in _RULES:
        if any(leaf.startswith(k) for k in keys):
            return segment
    return FALLBACK_SEGMENT


def assign_vertices(joint_indices, joint_weights, elem_size: int,
                    joints: list[str]) -> list[str]:
    """每個頂點依**權重最大**的骨骼決定歸屬。

    Args:
        joint_indices: 攤平的骨骼索引，長度 = 頂點數 × elem_size。
        joint_weights: 對應權重，同長度。
        elem_size:     每個頂點綁幾根骨（此資產為 3~7）。
        joints:        骨架的骨骼路徑清單。
    """
    segs: list[str] = []
    for v in range(len(joint_indices) // elem_size):
        lo = v * elem_size
        ws = joint_weights[lo:lo + elem_size]
        best = max(range(elem_size), key=lambda k: ws[k])
        if ws[best] <= 0.0:
            segs.append(FALLBACK_SEGMENT)
            continue
        ji = joint_indices[lo + best]
        segs.append(joint_to_segment(joints[ji]) if 0 <= ji < len(joints)
                    else FALLBACK_SEGMENT)
    return segs


def assign_faces(face_vertex_counts, face_vertex_indices,
                 vertex_segments: list[str]) -> list[str]:
    """每個面整片歸「多數頂點」所屬的部位。

    整片歸屬（而不是逐頂點切）是為了不讓同一個三角形同時出現在兩塊碰撞體裡 ——
    重複覆蓋會讓光達在接縫處量到兩層回波。平手時取 SEGMENTS 的固定順序，
    確保每次建出來的碰撞體完全一樣。
    """
    out: list[str] = []
    cursor = 0
    for n in face_vertex_counts:
        vs = face_vertex_indices[cursor:cursor + n]
        cursor += n
        tally = Counter(vertex_segments[i] for i in vs
                        if 0 <= i < len(vertex_segments))
        if not tally:
            out.append(FALLBACK_SEGMENT)
            continue
        top = max(tally.values())
        winners = [s for s, c in tally.items() if c == top]
        out.append(min(winners, key=lambda s: SEGMENTS.index(s)
                       if s in SEGMENTS else len(SEGMENTS)))
    return out
