"""每一趟（run01~04）的場景變體：障礙與行人**數量遞增、位置各不相同**。

2026-09-22 使用者指定：
  * 靜態障礙不要排成一直線，要沿走廊左右**交錯**（車必須蛇行）
  * 障礙與行人數量 run01 → run04 **依序更多**
  * 每一趟的生成位置都不一樣（不再是同一場景重複四次）

## 為什麼沿「路線中心線」而不是固定 y

走廊的中心線是斜的。從 map/4v3F.pgm（2026-09-22 實測）量到的自由帶：

    map x     自由 y 範圍      寬度
      +2    3.70 ~ 8.55      4.85
      -5    2.45 ~ 7.95      5.50
     -10    1.80 ~ 6.85      5.05
     -15    1.15 ~ 7.10      5.95
     -20    0.60 ~ 6.25      5.65

用固定 y 放障礙，在走廊一端會貼牆、另一端會離路線太遠。所以位置一律
用「沿路線中心線走多遠 + 垂直偏移多少」表示。

中心線取導航實際走的 routing 站點（從錄到的 /global_path 確認它真的
依序經過 c28 → c4 → c26 → c27）。刻意不含 x>-1 的路口，那裡是車的
出生區、也是地圖上往 y<0 的開口。

## 隨機但可重現

亂數種子只由 run_index 決定，所以第一遍（導航）與第二遍（回放算圖）
以及事後重跑都會得到**完全相同**的場景。
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from ros_graph_spec import CharacterWalk, Obstacle

#: 路線中心線（map frame）。取自導航實際經過的 routing 站點。
CORRIDOR_SPINE: tuple[tuple[float, float], ...] = (
    (-0.06, 5.95),      # c28 出發點
    (-3.05, 5.55),      # c4
    (-10.73, 4.51),     # c26
    (-16.93, 3.59),     # c27
    (-20.75, 3.17),     # c36（2026-09-23 延伸；終點）
)

#: 障礙可放的弧長區間（沿中心線，m）。
#: 中心線 c28→c4→c26→c27 全長約 17.0 m，這裡用掉 2.0~16.0 = 14 m，
#: 只避開出發點附近（路口、車的出生區）與最深處端點。
#: ⚠ 2026-09-22 原本只用 2.5~15.0，使用者指出「走廊很長，應該可以平均放」——
#:   縮在 12.5 m 裡放 6 個會變成連續 S 彎，不是走廊裡零星的障礙。
#: ⚠ 2026-09-23 路線終點由側邊的 c25 改成 spine 盡頭的 **c27**（s≈17.0）。
#:   上限留在 16.0 的話障礙離終點只剩 1.03 m，違反 NODE_CLEARANCE_M=1.8，
#:   `_place_clear_of_nodes` 會擺不下而丟例外。收到 14.5 保留 2.5 m 餘裕。
#: ⚠ 2026-09-23 終點再延伸到 c36（spine 全長 17.0 → 20.9 m），上限改到終點。
#:   使用者要求「主要以 c28→c27 為主」—— 這由幾何自然達成，不必硬切：
#:   站點死區（見 feasible_intervals）使可擺放長度約 80% 在 c28→c27、
#:   20% 在 c27→c36（[18.1, 19.9]）。離 c27、c36 的淨空由 node_clearance_ok 保證。
#:   （我先前以為 c27→c36「兩端各要 1.8 m、擺不下」—— 那是只沿中心線算，
#:    沒算障礙在側邊 1.25~1.5 m 時離站點的直線距離會變大。）
_SPINE_LEN = sum(math.dist(a, b) for a, b in zip(CORRIDOR_SPINE, CORRIDOR_SPINE[1:]))
OBSTACLE_S_RANGE = (2.0, round(_SPINE_LEN, 2))

#: 分層抽樣的抖動上限（佔格寬的比例，對格中心左右各這麼多）。
#: 實際抖動會再被 MIN_OBSTACLE_GAP_M 收窄（見 _jitter_fraction）。
STRATA_MAX_JITTER = 0.25

#: 零星障礙裡「道具 : 行人」的比例。用**平衡的隨機**：先決定各幾個，
#: 再隨機決定哪一格是哪一種 —— 純隨機在 run1（只有 1 個零星障礙）
#: 有一半機率抽不到任何道具，那一趟就完全看不到道具多樣性。
PROP_SHARE = 0.5

#: 肩並肩的一對「站著的人」：刻意封住走廊的**一側**，逼車走另一邊。
#: 這比一長串左右交錯的單一障礙更接近真實走廊，也更能考驗繞行決策。
#: 值是（弧長位置佔全長的比例, 兩人中心的側向位置, 兩人間距）。
SHOULDER_PAIR_AT_RATIO = 0.5
SHOULDER_PAIR_LATERAL_M = 1.05
SHOULDER_PAIR_SPACING_M = 0.62

#: 障礙的垂直偏移量（m）。左右交錯用 ±這個值。
#: 走廊最窄處自由帶約 4.85 m、中心線兩側各約 2.4 m；
#: 偏 1.25 m 後障礙外緣約在 1.65 m，車（半徑 0.35）仍有 0.4 m 以上可過。
OBSTACLE_LATERAL_M = 1.25

#: 各 run 的數量（依序更多）。
#: 障礙上限 6：弧長只有 12.5 m，再多就違反 MIN_OBSTACLE_GAP_M（見該常數說明）。
OBSTACLE_COUNTS: tuple[int, ...] = (3, 4, 5, 6)
WALKER_COUNTS: tuple[int, ...] = (2, 4, 6, 8)

#: 可以拿來當行人的角色。Character_09（貼在車後擋鏡頭）與
#: Character_15（站在 routing 點 c24 上）不列入，執行期本來也會被停用。
WALKER_POOL: tuple[str, ...] = (
    "Character_10", "Character_11", "Character_12", "Character_13",
    "Character_19", "Character_02", "Character_03", "Character_04",
    "Character_05", "Character_06",
)

#: 可以拿來當「站著的人」的角色池。與 WALKER_POOL 有重疊，
#: variant() 會先分配 walker，剩下的才給站著的，不會撞號。
STANDING_POOL: tuple[str, ...] = (
    "Character_14", "Character_16", "Character_17", "Character_18",
    "Character", "Character_01", "Character_07", "Character_08",
    "Character_02", "Character_03", "Character_04", "Character_05",
    "Character_06",
)

#: 行人速度池（m/s，皆 ≤1.0）。刻意挑成彼此的來回週期不成整數倍。
WALKER_SPEEDS: tuple[float, ...] = (1.0, 0.85, 0.7, 0.95, 0.6, 0.8, 0.75, 0.9)

#: 障礙彼此沿中心線的最小間隔（m）。
#: ⚠ 2026-09-22 先用 1.4，實跑出問題：9 個障礙左右交錯、縱向間距 2.4~3.0 m，
#:   車在 x≈-8 被夾住（最近障礙**中位** 0.48 m、VO 煞了 30%、車速 0.248、未達）。
#:   訓練的窄縫是 1.3~1.6 m 寬但**直的**，不是連續蛇行，這超出分布太多。
#:   改 2.2 m 並把數量上限收到 6，蛇行仍然存在但過得去。
MIN_OBSTACLE_GAP_M = 2.2

#: 只有「車真的會去」的站點需要淨空。
#:
#: ⚠ 對全部 29 個 routing 站要求淨空是錯的：走廊上到處是側室站點
#: （c1/c2/c5/c6/c9/c10…），1.8 m 淨空會讓障礙根本排不下。
#: 那些站點車不會去，障礙擺旁邊不擋任何事。
#: 這裡列的是錄到的 /global_path **實際依序經過**的站。
#: 唯一定義在 `ros_graph_spec.ROUTE_WAYPOINTS`，不要在這裡另寫一份 ——
#: 2026-09-23 路線由 c28↔c25 改成 c28↔c27，兩處各寫一份就會有一邊沒改到。
def _route_node_names() -> tuple[str, ...]:
    import ros_graph_spec as _S
    return _S.ROUTE_WAYPOINTS


ROUTE_NODE_NAMES: tuple[str, ...] = _route_node_names()

#: 障礙與**路線上**站點的最小淨空（m）。
#: ⚠ 2026-09-22 先用 0.9，實跑出問題：ped_4_7 落在終點 c25 旁 1.25 m，
#:   而抵達判定半徑是 1.0 m（monitor_navigation.ARRIVE_RADIUS_M）——
#:   障礙擠在目標旁邊，車到不了是設定造成的，不是能力問題。
#: 1.8 = 抵達半徑 1.0 + 車半徑 0.35 + 障礙半徑 0.3 + 餘裕。
NODE_CLEARANCE_M = 1.8

#: 障礙／站立人物與**任何一個** routing 站點的最小淨空（m）。
#:
#: ⚠ 2026-09-23 使用者要求「靜態行人不可以站在要導航的點位之 1.0 m 附近」。
#: 實測 run4 的站立人物 `Character` 離 c2 只有 0.616 m。
#: 1.0 = monitor_navigation.ARRIVE_RADIUS_M —— 抵達判定半徑內不該站人。
#: 路線上的站另外要求更嚴的 NODE_CLEARANCE_M（1.8）；這條管的是**全部 29 站**，
#: 因為那些側室站點雖然這次不走，換一條路線就會走到。
ANY_NODE_CLEARANCE_M = 1.0


#: 行人**起點**彼此（以及與障礙）的最小間距（m）。
#:
#: ⚠⚠ 2026-09-23 使用者回報「行人之間會互相穿透」。實測：最小間距一律發生在
#: `sim_t=0.03s`（第一幀），數值正好等於這裡產生的起點距離
#: （run02 0.130→0.160、run03 0.206→0.231、run04 0.496→0.521 m）。
#: 不是 ORCA 壞了 —— ORCA 只保證「從不重疊的狀態開始」不會撞，
#: **解不開一開始就重疊的狀態**。所以起點自己不能疊。
#: 0.85 = 2 × 行人半徑 0.30 + 0.25 餘裕（第一步還會再靠近一點）。
MIN_WALKER_START_GAP_M = 0.85

#: 找不重疊起點的重抽次數。8 個人在 17 m 走廊裡要 0.85 m 間距很寬鬆，
#: 抽不到就是設定有問題，要**大聲失敗**而不是放一個重疊的起點。
MAX_START_ATTEMPTS = 60


def _far_enough(p, others, gap: float) -> bool:
    """``p`` 是否離 ``others`` 每一個都至少 ``gap``。

    ``others`` 的元素可以是 ``(x, y)`` 或 ``(x, y, 額外半徑)``；
    後者要求的距離是 ``gap + 額外半徑``（大道具要離更遠）。
    """
    for q in others:
        extra = q[2] if len(q) > 2 else 0.0
        if math.dist(p, q[:2]) < gap + extra:
            return False
    return True


def spine_length() -> float:
    return sum(math.dist(CORRIDOR_SPINE[i], CORRIDOR_SPINE[i + 1])
               for i in range(len(CORRIDOR_SPINE) - 1))


def spine_point(s: float):
    """沿中心線走 ``s`` 公尺的位置與切線方向 ``(x, y, heading_rad)``。

    超出兩端時夾住（不外插，否則障礙會被放到走廊外面）。
    """
    s = max(0.0, min(s, spine_length()))
    acc = 0.0
    for i in range(len(CORRIDOR_SPINE) - 1):
        a, b = CORRIDOR_SPINE[i], CORRIDOR_SPINE[i + 1]
        seg = math.dist(a, b)
        if acc + seg >= s or i == len(CORRIDOR_SPINE) - 2:
            u = 0.0 if seg == 0 else (s - acc) / seg
            u = max(0.0, min(1.0, u))
            hd = math.atan2(b[1] - a[1], b[0] - a[0])
            return (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, hd)
        acc += seg
    a = CORRIDOR_SPINE[-1]
    return (a[0], a[1], 0.0)


def offset_from_spine(s: float, lateral: float):
    """中心線上 ``s`` 處、往左（正）或右（負）偏 ``lateral`` 公尺的點。"""
    x, y, hd = spine_point(s)
    return (x - lateral * math.sin(hd), y + lateral * math.cos(hd))


@dataclass(frozen=True)
class StandingPerson:
    """站著不動的行人：擺在某個「人形障礙」的位置上。

    為什麼要人物疊在障礙上：障礙本身是 USD 裡的圓柱，**物理正確**
    （靜態碰撞體，會真的擋住車），但外觀是圓柱。使用者要看到真人。
    所以圓柱設成不可見、只留碰撞，再把一個 People 角色擺在同一點負責外觀，
    角色本身的逐部位碰撞體也讓光達打到人的輪廓。
    """

    name: str
    map_x: float
    map_y: float
    yaw: float


@dataclass(frozen=True)
class SceneVariant:
    """一趟的場景。"""

    run_index: int
    obstacles: tuple[Obstacle, ...]
    walks: tuple[CharacterWalk, ...]
    standing: tuple[StandingPerson, ...]


def shoulder_pair_positions(rng: random.Random | None = None, s_mid=None):
    """回傳肩並肩那一對的兩個位置（同一側、沿走廊並排）。

    ``s_mid`` 不給就用 SHOULDER_PAIR_AT_RATIO（舊行為，留給測試與預覽）。
    分層抽樣時由 variant() 傳入那一對所屬格子抽到的位置。
    """
    if s_mid is None:
        s_mid = OBSTACLE_S_RANGE[0] + SHOULDER_PAIR_AT_RATIO * (
            OBSTACLE_S_RANGE[1] - OBSTACLE_S_RANGE[0])
    side = 1.0 if (rng is None or rng.random() < 0.5) else -1.0
    half = SHOULDER_PAIR_SPACING_M / 2.0
    return ((s_mid, side * (SHOULDER_PAIR_LATERAL_M - half)),
            (s_mid, side * (SHOULDER_PAIR_LATERAL_M + half)))


def _jitter_fraction(width: float) -> float:
    """格寬 ``width`` 時能用多大的抖動（佔格寬比例），保證相鄰間隔 >= MIN_OBSTACLE_GAP_M。

    相鄰兩格各往內抖 j·w 時最近：間隔 = w − 2·j·w。要 >= MIN_GAP
    → j <= (w − MIN_GAP) / (2w)。run1 格很寬、抖得多；run4 格窄、幾乎等距。
    """
    if width <= MIN_OBSTACLE_GAP_M:
        return 0.0
    return min(STRATA_MAX_JITTER, (width - MIN_OBSTACLE_GAP_M) / (2.0 * width))


def strata(n_slots: int, lo: float, hi: float):
    """把 [lo, hi] 切成 ``n_slots`` 等份，回傳每格 (下界, 上界)。"""
    if n_slots < 1:
        raise ValueError(f"格數要 >= 1，收到 {n_slots}")
    w = (hi - lo) / n_slots
    return [(lo + i * w, lo + (i + 1) * w) for i in range(n_slots)]


def _slot_candidates(lo: float, hi: float, rng: random.Random):
    """一格裡的候選弧長：先試抖動抽到的，再以 0.2 m 步長往兩側找（不出格）。"""
    w = hi - lo
    first = (lo + hi) / 2.0 + rng.uniform(-1.0, 1.0) * _jitter_fraction(w) * w
    out = [first]
    for k in range(1, int(w / 0.2) + 2):
        for d in (0.2 * k, -0.2 * k):
            c = first + d
            if lo <= c <= hi:
                out.append(c)
    return out


def _obstacle_positions(n: int, rng: random.Random):
    """（舊介面，保留給預覽）回傳 [(s, 側向, tag)]，不含站點淨空處理。"""
    n_slots = max(1, n - 1)
    slots = strata(n_slots, *OBSTACLE_S_RANGE)
    pair_slot = n_slots // 2
    out = []
    k = 0
    for i, (lo, hi) in enumerate(slots):
        sc = _slot_candidates(lo, hi, rng)[0]
        if i == pair_slot:
            for sp in shoulder_pair_positions(rng, sc):
                out.append((sp[0], sp[1], "pair"))
        else:
            side = 1.0 if k % 2 == 0 else -1.0
            out.append((sc, side * OBSTACLE_LATERAL_M, "single"))
            k += 1
    return out


def route_nodes(stations):
    """從站點表挑出路線上的那幾站。缺的站直接略過（表換了不要炸）。"""
    return {n: stations[n] for n in ROUTE_NODE_NAMES if n in stations}


def _nearest_node_dist(p, stations) -> float:
    if not stations:
        return float("inf")
    return min(math.hypot(v[0] - p[0], v[1] - p[1]) for v in stations.values())


#: NODE_CLEARANCE_M 的 1.8 裡已經含一個 0.3 m 的障礙半徑。
_NODE_CLEARANCE_ASSUMED_R = 0.3


def node_clearance_ok(p, on_route, all_nodes, extent: float = 0.25) -> bool:
    """中心在 ``p``、外接半徑 ``extent`` 的障礙是否同時滿足兩條淨空要求。

    * 路線上的站：>= NODE_CLEARANCE_M（1.8，已含 0.3 m 障礙半徑）
      —— 大於 0.3 的部分另外加上去。
    * **任何**站：人的**中心** >= ANY_NODE_CLEARANCE_M（1.0，使用者原話是
      「靜態行人不可站在點位 1.0 m 附近」）；道具以**最近的邊**算，
      即中心 >= 1.0 + 外接半徑。

    ⚠ 道具不是點：SM_Cupboard 長 1.84 m，只看中心會讓一端壓到站點。
    """
    route_need = NODE_CLEARANCE_M + max(0.0, extent - _NODE_CLEARANCE_ASSUMED_R)
    any_need = ANY_NODE_CLEARANCE_M + (extent if extent > 0.25 else 0.0)
    return (_nearest_node_dist(p, on_route) >= route_need
            and _nearest_node_dist(p, all_nodes) >= any_need)


def _place_clear_of_nodes(s: float, lat: float, on_route, all_nodes=None):
    """把候選位置推離 routing 站點，**保留左右側別**（交錯不能被破壞）。

    先沿中心線前後挪（不改側別），再退而縮小側向距離。
    """
    if all_nodes is None:
        all_nodes = on_route
    base = offset_from_spine(s, lat)
    if node_clearance_ok(base, on_route, all_nodes):
        return base
    side = 1.0 if lat > 0 else -1.0
    s0, s1 = OBSTACLE_S_RANGE
    # 沿走廊前後挪 ±4 m（保留側別），每個位置再試幾種側向距離。
    for step in [i * 0.4 for i in range(1, 11)]:
        for ds in (step, -step):
            cs = max(s0, min(s1, s + ds))
            for mag in (abs(lat), 1.5, 1.6, 1.1, 0.85):
                cand = offset_from_spine(cs, side * mag)
                if node_clearance_ok(cand, on_route, all_nodes):
                    return cand
    # ⚠ 找不到就大聲報錯，不要默默把障礙擺在目標旁邊。
    #   2026-09-22 就是因為這裡靜靜回傳原值，ped_4_7 落在終點 c25 旁 1.25 m，
    #   整段導航 FAIL，而且要等跑完才看得出來。
    raise RuntimeError(
        f"排不出同時離路線站點 {NODE_CLEARANCE_M} m、離任何站點 "
        f"{ANY_NODE_CLEARANCE_M} m 以上的障礙位置"
        f"（弧長 {s:.2f}、側向 {lat:+.2f}）")


#: 對側相鄰障礙的最小間隔 = MIN_OBSTACLE_GAP_M × 這個比例。
#: 左右交錯本來就是要車蛇行，對側不必像同側那樣拉開（沿用 2026-09-22 的規則）。
OPPOSITE_SIDE_GAP_RATIO = 0.7

#: 可擺放區間的取樣步長（m）。
_FEASIBLE_STEP_M = 0.05


def feasible_intervals(on_route, stations, extent: float = 0.25,
                       s_range=None, step: float = _FEASIBLE_STEP_M):
    """回傳 ``[(a, b), ...]``：在這些弧長上，左右**至少一側**擺得下外接半徑
    ``extent`` 的障礙（滿足兩條站點淨空）。

    ⚠⚠ 2026-09-23：走廊上有擺不下任何東西的**死區**——起點附近
    （路線站 c28、c4 + 側室 c1、c2、c10、c37）、c26 附近（**兩側各有一個
    側室站 c9、c5**，左右都被封）。按弧長等分的話，落在死區的格子會被擠壞，
    並排那一對在 run3 就因此擺不下。所以要按「可擺放長度」等分。
    """
    lo, hi = s_range or OBSTACLE_S_RANGE
    mags = (OBSTACLE_LATERAL_M, 1.5, 1.1, 0.85)
    out, cur = [], None
    n = int(round((hi - lo) / step))
    for j in range(n + 1):
        sv = lo + j * step
        ok = any(node_clearance_ok(offset_from_spine(sv, side * m), on_route,
                                   stations, extent)
                 for side in (1.0, -1.0) for m in mags)
        if ok and cur is None:
            cur = sv
        elif not ok and cur is not None:
            out.append((cur, sv - step))
            cur = None
    if cur is not None:
        out.append((cur, hi))
    return out


def _measure_to_s(intervals, t: float) -> float:
    """可擺放長度上的位置 ``t`` → 弧長 s。"""
    for a, b in intervals:
        if t <= b - a:
            return a + t
        t -= b - a
    return intervals[-1][1]


def measure_slots(intervals, n_slots: int):
    """把可擺放的**總長**等分成 ``n_slots`` 格，回傳每格的 (起, 迄)（在可擺放長度上）。"""
    total = sum(b - a for a, b in intervals)
    if total <= 0:
        raise RuntimeError("整條走廊沒有任何可擺放的位置")
    w = total / n_slots
    return [(i * w, (i + 1) * w) for i in range(n_slots)], total


def _gap_ok(prev, sc: float, side: float) -> bool:
    """與前一個障礙的沿走廊間隔夠不夠。同側要 MIN_OBSTACLE_GAP_M，對側打 7 折。"""
    if prev is None:
        return True
    ps, pside = prev
    need = MIN_OBSTACLE_GAP_M if pside == side else MIN_OBSTACLE_GAP_M * OPPOSITE_SIDE_GAP_RATIO
    return sc - ps >= need


def _slot_s_candidates(intervals, t0: float, t1: float, rng: random.Random):
    """一格（可擺放長度 [t0, t1]）的候選弧長：先試抖動抽到的，再往兩側掃。"""
    w = t1 - t0
    j = _jitter_fraction(w)
    t_first = (t0 + t1) / 2.0 + rng.uniform(-1.0, 1.0) * j * w
    ts = [t_first]
    k = 1
    while True:
        added = False
        for d in (0.1 * k, -0.1 * k):
            t = t_first + d
            if t0 <= t <= t1:
                ts.append(t)
                added = True
        if not added:
            break
        k += 1
    return [_measure_to_s(intervals, t) for t in ts]


def _place_obstacles(n: int, rng: random.Random, on_route, stations,
                     run_index: int) -> list[Obstacle]:
    """分層抽樣擺 ``n`` 個靜態障礙：**每格一個**，並排那一對佔中間一格。

    ⚠⚠ 2026-09-23 使用者指出分布不平均，實測原本的做法（全長等分 + 抖動，
    再把並排那一對固定在正中、撞到就推開 2.2 m，最後站點淨空再挪 ±4 m）
    嚴重結塊：run1 的 3 個障礙擠在 5.1~8.2 m，後面 8.8 m 全空。改成：

    * 先算出**可擺放區間**（扣掉站點死區），把可擺放的**總長**切成 n−1 格
      （並排那一對共用一格），**每格恰好一個**
    * 格內抖動自動收窄，保證相鄰間隔（同側 2.2 m、對側 1.54 m）
    * 站點淨空不夠時**只在格內**找，不跨格 —— 跨格就又結塊了
    * 零星障礙用平衡的隨機決定是道具還是站立行人；抽到的道具擺不下就
      換小一點的道具，全部擺不下才退成站立行人

    找不到位置就**大聲失敗**，不要默默擺在站點旁邊。
    """
    import props as P

    intervals = feasible_intervals(on_route, stations, 0.25)
    n_slots = max(1, n - 1)
    slots, _total = measure_slots(intervals, n_slots)
    pair_slot = n_slots // 2
    n_singles = n - 2
    n_props = math.ceil(n_singles * PROP_SHARE) if n_singles > 0 else 0
    kinds = ["prop"] * n_props + ["person"] * (n_singles - n_props)
    rng.shuffle(kinds)

    out: list[Obstacle] = []
    prev = None
    single_k = 0
    idx = 0
    mags_default = (OBSTACLE_LATERAL_M, 1.5, 1.1, 0.85)
    for i, (t0, t1) in enumerate(slots):
        cands = _slot_s_candidates(intervals, t0, t1, rng)
        if i == pair_slot:
            side = 1.0 if rng.random() < 0.5 else -1.0
            half = SHOULDER_PAIR_SPACING_M / 2.0
            lats = (side * (SHOULDER_PAIR_LATERAL_M - half),
                    side * (SHOULDER_PAIR_LATERAL_M + half))
            placed = None
            for sc in cands:
                if not _gap_ok(prev, sc, side):
                    continue
                pts = [offset_from_spine(sc, la) for la in lats]
                if all(node_clearance_ok(q, on_route, stations, 0.25) for q in pts):
                    placed = (sc, pts)
                    break
            if placed is None:
                raise RuntimeError(
                    f"run{run_index} 第 {i} 格擺不下並排那一對")
            sc, pts = placed
            for q in pts:
                out.append(Obstacle(f"pair_{run_index}_{idx}", round(q[0], 3),
                                    round(q[1], 3), "person"))
                idx += 1
            prev = (sc, side)
            continue

        kind = kinds[single_k]
        side = 1.0 if single_k % 2 == 0 else -1.0
        single_k += 1
        base_lat = OBSTACLE_LATERAL_M * rng.uniform(0.85, 1.0)
        # 抽到的道具先試；擺不下就換其他道具（隨機順序）；全部擺不下才退成人。
        # ⚠ 實測 run3 第一格擺不下外接半徑 1.01 m 的大盆栽，換檔案櫃就擺得下。
        if kind == "prop":
            first = rng.choice(P.PROPS)
            rest = [x for x in P.PROPS if x is not first]
            rng.shuffle(rest)
            options = [first] + rest + [None]
        else:
            options = [None]
        placed = None
        for spec in options:
            extent = spec.extent_radius if spec else 0.25
            for sc in cands:
                if not _gap_ok(prev, sc, side):
                    continue
                for mag in (base_lat,) + mags_default[1:]:
                    q = offset_from_spine(sc, side * mag)
                    if node_clearance_ok(q, on_route, stations, extent):
                        placed = (sc, q, spec)
                        break
                if placed:
                    break
            if placed:
                break
        if placed is None:
            raise RuntimeError(
                f"run{run_index} 第 {i} 格連站立行人都擺不下"
                f"——同時要離路線站點 >= {NODE_CLEARANCE_M} m、離任何站點 "
                f">= {ANY_NODE_CLEARANCE_M} m、離前一個障礙夠遠")
        sc, q, spec = placed
        if spec:
            heading = spine_point(sc)[2]
            yaw = math.degrees(P.yaw_along(heading, spec))
            out.append(Obstacle(f"prop_{run_index}_{idx}", round(q[0], 3),
                                round(q[1], 3), "prop",
                                height=round(spec.height, 3),
                                size_x=round(spec.size_x, 3),
                                size_y=round(spec.size_y, 3),
                                yaw_deg=round(yaw, 2), asset=spec.name))
        else:
            out.append(Obstacle(f"ped_{run_index}_{idx}", round(q[0], 3),
                                round(q[1], 3), "person"))
        idx += 1
        prev = (sc, side)
    return out


def variant(run_index: int, stations=None) -> SceneVariant:
    """產生第 ``run_index`` 趟（1 起算）的場景。同一個 index 永遠一樣。"""
    if run_index < 1:
        raise ValueError(f"run_index 從 1 起算，收到 {run_index}")
    if stations is None:
        import ros_graph_spec as S
        stations = S.read_station_nodes(S.ROUTING_STATION_JSON)
    k = min(run_index, len(OBSTACLE_COUNTS)) - 1
    rng = random.Random(9000 + run_index)
    on_route = route_nodes(stations)

    obs = _place_obstacles(OBSTACLE_COUNTS[k], rng, on_route, stations,
                           run_index)

    n_walk = WALKER_COUNTS[k]
    walks = []
    # 起點也不能壓在障礙上。第三個值是「比一個人多出來的外接半徑」——
    # 道具（最大的盆栽外接半徑 1.01 m）比人（0.25）大得多，只看中心會壓進去。
    taken = [(o.map_x, o.map_y, max(0.0, o.extent_radius - 0.25)) for o in obs]
    obstacle_only = list(taken)
    for i in range(n_walk):
        name = WALKER_POOL[i % len(WALKER_POOL)]
        spd = WALKER_SPEEDS[i % len(WALKER_SPEEDS)]
        a = b = None
        for _ in range(MAX_START_ATTEMPTS):
            if i % 3 == 2:                   # 橫穿
                s = rng.uniform(4.0, 13.0)
                cand_a = offset_from_spine(s, 1.6)
                cand_b = offset_from_spine(s, -1.6)
            else:                            # 沿走廊
                s0 = rng.uniform(1.0, 5.0)
                s1 = s0 + rng.uniform(8.0, 14.0)
                lat = rng.choice((0.75, -0.75, 1.15, -1.15))
                cand_a = offset_from_spine(s0, lat)
                cand_b = offset_from_spine(min(s1, spine_length() - 0.5), lat)
            # ⚠ 要檢查**捨入後**的座標。waypoints 存的是小數 3 位，
            #   拿未捨入的值檢查會讓 0.8500 存成 0.8494 —— 差一點點，
            #   但那正是「剛好過門檻」的那一對，等於檢查沒生效。
            cand_a = tuple(round(v, 3) for v in cand_a)
            cand_b = tuple(round(v, 3) for v in cand_b)
            # 終點也要避開障礙：終點壓在大道具裡，ORCA 行人會卡在那裡一直推。
            if (_far_enough(cand_a, taken, MIN_WALKER_START_GAP_M)
                    and _far_enough(cand_b, obstacle_only, MIN_WALKER_START_GAP_M)):
                a, b = cand_a, cand_b
                break
        if a is None:
            raise RuntimeError(
                f"run{run_index} 第 {i} 個行人試了 {MAX_START_ATTEMPTS} 次都找不到"
                f"離其他人/障礙 {MIN_WALKER_START_GAP_M} m 以上的起點")
        taken.append((a[0], a[1], 0.0))
        walks.append(CharacterWalk(
            name, (a, b),
            speed=spd, phase_s=round(rng.uniform(0.0, 12.0), 2)))

    # 站著的人：一個角色對應一個「人形障礙」，擺在同一個位置。
    # 朝向取垂直於中心線（面向走廊另一側），輪廓比較寬、也比較像在擋路。
    pool = [n for n in STANDING_POOL if n not in {w.name for w in walks}]
    rng.shuffle(pool)
    standing = []
    for o in obs:
        if o.kind != "person" or not pool:
            continue
        # 找最近的中心線點，取其法線方向當朝向
        best_hd, bd = 0.0, 1e9
        for j in range(0, int(spine_length() * 5) + 1):
            ss = j / 5.0
            px, py, hd = spine_point(ss)
            d = math.hypot(px - o.map_x, py - o.map_y)
            if d < bd:
                bd, best_hd = d, hd
        # 面向中心線那一側
        px, py, hd = spine_point(max(0.0, min(spine_length(), 0.0)))
        yaw = best_hd + (math.pi / 2.0 if o.map_y < 0 else -math.pi / 2.0)
        standing.append(StandingPerson(pool.pop(), o.map_x, o.map_y,
                                       round(yaw, 4)))
    return SceneVariant(run_index, tuple(obs), tuple(walks), tuple(standing))


def all_variant_obstacles(n_runs: int = 4, stations=None):
    """所有變體用到的障礙聯集，供離線寫進 USD。

    為什麼要全部寫進 USD 而不是執行期建：執行期新建的 prim 不保證進算圖
    （2026-09-22 的教訓：執行期建的燈完全不進 RTX）。全部寫進 USD、
    執行期只用 ``SetActive`` 開關，是已經驗證過可靠的做法。
    名字帶 run 編號，所以不同變體不會互撞。
    """
    out = []
    for i in range(1, n_runs + 1):
        out.extend(variant(i, stations=stations).obstacles)
    return tuple(out)
