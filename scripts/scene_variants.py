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
)

#: 障礙可放的弧長區間（沿中心線，m）。
#: 中心線 c28→c4→c26→c27 全長約 17.0 m，這裡用掉 2.0~16.0 = 14 m，
#: 只避開出發點附近（路口、車的出生區）與最深處端點。
#: ⚠ 2026-09-22 原本只用 2.5~15.0，使用者指出「走廊很長，應該可以平均放」——
#:   縮在 12.5 m 裡放 6 個會變成連續 S 彎，不是走廊裡零星的障礙。
OBSTACLE_S_RANGE = (2.0, 16.0)

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
#: 這裡列的是錄到的 /global_path **實際依序經過**的站：
#: c28（出發）→ c4 → c26 → c27，加上 c25（去程終點）。
ROUTE_NODE_NAMES: tuple[str, ...] = ("c28", "c4", "c26", "c27", "c25")

#: 障礙與**路線上**站點的最小淨空（m）。
#: ⚠ 2026-09-22 先用 0.9，實跑出問題：ped_4_7 落在終點 c25 旁 1.25 m，
#:   而抵達判定半徑是 1.0 m（monitor_navigation.ARRIVE_RADIUS_M）——
#:   障礙擠在目標旁邊，車到不了是設定造成的，不是能力問題。
#: 1.8 = 抵達半徑 1.0 + 車半徑 0.35 + 障礙半徑 0.3 + 餘裕。
NODE_CLEARANCE_M = 1.8


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


def shoulder_pair_positions(rng: random.Random | None = None):
    """回傳肩並肩那一對的兩個位置（同一側、沿走廊並排）。"""
    s_mid = OBSTACLE_S_RANGE[0] + SHOULDER_PAIR_AT_RATIO * (
        OBSTACLE_S_RANGE[1] - OBSTACLE_S_RANGE[0])
    side = 1.0 if (rng is None or rng.random() < 0.5) else -1.0
    half = SHOULDER_PAIR_SPACING_M / 2.0
    return ((s_mid, side * (SHOULDER_PAIR_LATERAL_M - half)),
            (s_mid, side * (SHOULDER_PAIR_LATERAL_M + half)))


def _obstacle_positions(n: int, rng: random.Random):
    """沿**整條**中心線等分後加抖動，左右交錯；中段插一對肩並肩。

    肩並肩那一對算在總數內（所以 n=3 時是「一對 + 一個」）。
    """
    s0, s1 = OBSTACLE_S_RANGE
    pair = shoulder_pair_positions(rng)
    n_single = max(0, n - 2)
    step = (s1 - s0) / max(1, n_single + 1)
    singles = []
    for i in range(n_single):
        s = s0 + step * (i + 1)
        s += rng.uniform(-0.3, 0.3) * step
        s = max(s0, min(s1, s))
        # 不要跟肩並肩那一對重疊
        if abs(s - pair[0][0]) < MIN_OBSTACLE_GAP_M:
            s += MIN_OBSTACLE_GAP_M * (1 if s >= pair[0][0] else -1)
            s = max(s0, min(s1, s))
        side = 1.0 if i % 2 == 0 else -1.0
        singles.append((s, side * OBSTACLE_LATERAL_M * rng.uniform(0.85, 1.0)))
    tagged = [(sp[0], sp[1], "pair") for sp in pair] + \
             [(sg[0], sg[1], "single") for sg in singles]
    out = sorted(tagged, key=lambda it: it[0])
    # 同側相鄰太近才推開；對側相鄰不必（那正是交錯要的效果）
    for i in range(1, len(out)):
        if out[i][2] == "pair" and out[i - 1][2] == "pair":
            continue                     # 並排的一對刻意靠在一起，不要推開
        same_side = (out[i][1] > 0) == (out[i - 1][1] > 0)
        need = MIN_OBSTACLE_GAP_M if same_side else MIN_OBSTACLE_GAP_M * 0.7
        if out[i][0] - out[i - 1][0] < need and out[i][0] != out[i - 1][0]:
            out[i] = (min(s1, out[i - 1][0] + need), out[i][1], out[i][2])
    return out


def route_nodes(stations):
    """從站點表挑出路線上的那幾站。缺的站直接略過（表換了不要炸）。"""
    return {n: stations[n] for n in ROUTE_NODE_NAMES if n in stations}


def _nearest_node_dist(p, stations) -> float:
    if not stations:
        return float("inf")
    return min(math.hypot(v[0] - p[0], v[1] - p[1]) for v in stations.values())


def _place_clear_of_nodes(s: float, lat: float, stations):
    """把候選位置推離 routing 站點，**保留左右側別**（交錯不能被破壞）。

    先沿中心線前後挪（不改側別），再退而縮小側向距離。
    """
    base = offset_from_spine(s, lat)
    if _nearest_node_dist(base, stations) >= NODE_CLEARANCE_M:
        return base
    side = 1.0 if lat > 0 else -1.0
    s0, s1 = OBSTACLE_S_RANGE
    # 沿走廊前後挪 ±4 m（保留側別），每個位置再試幾種側向距離。
    for step in [i * 0.4 for i in range(1, 11)]:
        for ds in (step, -step):
            cs = max(s0, min(s1, s + ds))
            for mag in (abs(lat), 1.5, 1.6, 1.1, 0.85):
                cand = offset_from_spine(cs, side * mag)
                if _nearest_node_dist(cand, stations) >= NODE_CLEARANCE_M:
                    return cand
    # ⚠ 找不到就大聲報錯，不要默默把障礙擺在目標旁邊。
    #   2026-09-22 就是因為這裡靜靜回傳原值，ped_4_7 落在終點 c25 旁 1.25 m，
    #   整段導航 FAIL，而且要等跑完才看得出來。
    raise RuntimeError(
        f"排不出離路線站點 {NODE_CLEARANCE_M} m 以上的障礙位置"
        f"（弧長 {s:.2f}、側向 {lat:+.2f}）")


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

    raw = _obstacle_positions(OBSTACLE_COUNTS[k], rng)

    # 並排的一對要**整體**平移才會維持並排。逐個推開的話就散了。
    pair_idx = [i for i, it in enumerate(raw) if it[2] == "pair"]
    if len(pair_idx) == 2:
        s_pair = raw[pair_idx[0]][0]
        ds_ok = 0.0
        for ds in (0.0, 0.8, -0.8, 1.6, -1.6, 2.4, -2.4, 3.2, -3.2):
            if all(_nearest_node_dist(
                    offset_from_spine(max(0.0, s_pair + ds), raw[i][1]),
                    on_route) >= NODE_CLEARANCE_M for i in pair_idx):
                ds_ok = ds
                break
        for i in pair_idx:
            raw[i] = (max(0.0, s_pair + ds_ok), raw[i][1], "pair")

    obs = []
    n_single = 0
    for i, (s, lat, tag) in enumerate(raw):
        if tag == "pair":
            x, y = offset_from_spine(s, lat)
            obs.append(Obstacle(f"pair_{run_index}_{i}", round(x, 3), round(y, 3),
                                "person"))      # 並排的一定是兩個人
            continue
        x, y = _place_clear_of_nodes(s, lat, on_route)
        if n_single % 3 == 2:   # 零星障礙裡每三個有一個是推車（方箱）
            obs.append(Obstacle(f"box_{run_index}_{i}", round(x, 3), round(y, 3),
                                "box", size_x=0.7, size_y=0.5, height=1.55))
        else:
            obs.append(Obstacle(f"ped_{run_index}_{i}", round(x, 3), round(y, 3),
                                "person"))
        n_single += 1

    n_walk = WALKER_COUNTS[k]
    walks = []
    for i in range(n_walk):
        name = WALKER_POOL[i % len(WALKER_POOL)]
        spd = WALKER_SPEEDS[i % len(WALKER_SPEEDS)]
        if i % 3 == 2:                       # 橫穿
            s = rng.uniform(4.0, 13.0)
            a = offset_from_spine(s, 1.6)
            b = offset_from_spine(s, -1.6)
        else:                                # 沿走廊
            s0 = rng.uniform(1.0, 5.0)
            s1 = s0 + rng.uniform(8.0, 14.0)
            lat = rng.choice((0.75, -0.75, 1.15, -1.15))
            a = offset_from_spine(s0, lat)
            b = offset_from_spine(min(s1, spine_length() - 0.5), lat)
        walks.append(CharacterWalk(
            name, (tuple(round(v, 3) for v in a), tuple(round(v, 3) for v in b)),
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
