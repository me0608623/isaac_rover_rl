"""場景變體的測試。"""

from __future__ import annotations

import math

import pytest

from scene_variants import (CORRIDOR_SPINE, MIN_OBSTACLE_GAP_M, OBSTACLE_COUNTS,
                            WALKER_COUNTS, offset_from_spine, spine_length,
                            spine_point, variant)



#: 兩條路線 × 四趟。場景的不變量兩條路線都要成立 ——
#: 2026-09-23 加入第二條路線時，這些測試原本只測預設路線，c36 的障礙沒人把關。
import ros_graph_spec as _S_ROUTES
ALL_VARIANTS = [(rk, i) for rk in _S_ROUTES.ROUTE_ORDER for i in (1, 2, 3, 4)]

def test_counts_increase_with_run_index():
    """★ 使用者指定數量要依序更多。"""
    obs = [len(variant(i).obstacles) for i in (1, 2, 3, 4)]
    wk = [len(variant(i).walks) for i in (1, 2, 3, 4)]
    assert obs == list(OBSTACLE_COUNTS)
    assert wk == list(WALKER_COUNTS)
    assert obs == sorted(obs) and wk == sorted(wk)


def test_same_run_index_always_gives_the_same_scene():
    """★ 第一遍導航與第二遍回放算圖是兩個行程，場景必須逐字相同，
    否則影片裡的障礙與 rosbag 裡光達打到的對不起來。"""
    a, b = variant(3), variant(3)
    assert [(o.name, o.map_x, o.map_y) for o in a.obstacles] == \
           [(o.name, o.map_x, o.map_y) for o in b.obstacles]
    assert [(w.name, w.waypoints, w.speed, w.phase_s) for w in a.walks] == \
           [(w.name, w.waypoints, w.speed, w.phase_s) for w in b.walks]


def test_different_runs_give_different_positions():
    """★ 四趟不能是同一個場景重複四次。"""
    pos = [tuple((o.map_x, o.map_y) for o in variant(i).obstacles[:3])
           for i in (1, 2, 3, 4)]
    assert len(set(pos)) == 4


def _sides(obstacles):
    """每個障礙在中心線的哪一側（+1 左 / -1 右）。"""
    out = []
    for o in obstacles:
        best, bs = None, 1e9
        for j in range(0, int(spine_length() * 10) + 1):
            s = j / 10.0
            px, py, hd = spine_point(s)
            d = math.hypot(px - o.map_x, py - o.map_y)
            if d < bs:
                bs, best = d, (px, py, hd)
        px, py, hd = best
        cross = (-math.sin(hd)) * (o.map_x - px) + math.cos(hd) * (o.map_y - py)
        out.append(1 if cross > 0 else -1)
    return out


def test_there_is_exactly_one_shoulder_to_shoulder_pair():
    """★ 使用者指定：走廊中段要有**兩個人肩並肩站在一側**，逼車走另一邊。
    這比一長串左右交錯更接近真實走廊。"""
    for rk, run in ALL_VARIANTS:
        pair = [o for o in variant(run, route=rk).obstacles if o.name.startswith("pair_")]
        assert len(pair) == 2, f"{rk} run{run} 有 {len(pair)} 個並排障礙"
        d = math.hypot(pair[0].map_x - pair[1].map_x, pair[0].map_y - pair[1].map_y)
        assert 0.4 <= d <= 0.9, f"{rk} run{run} 兩人相距 {d:.2f} m，不像肩並肩"
        assert _sides(pair)[0] == _sides(pair)[1], f"{rk} run{run} 那一對不在同一側"
        assert all(o.kind == "person" for o in pair), "並排的必須是人，不是推車"


def test_single_obstacles_alternate_sides():
    """★ 零星障礙要左右交錯（並排那一對不算，它刻意同側）。"""
    for rk, run in ALL_VARIANTS:
        v = variant(run, route=rk)
        singles = [o for o in v.obstacles if not o.name.startswith("pair_")]
        if len(singles) < 2:
            continue
        sides = _sides(singles)
        flips = sum(1 for a, b in zip(sides, sides[1:]) if a != b)
        assert flips == len(sides) - 1, f"{rk} run{run} 零星障礙側邊 {sides} 沒交錯"


def test_obstacles_are_spread_over_the_whole_corridor():
    """★ 使用者指出「走廊很長，應該可以平均放置」——
    最難那趟的障礙跨距要涵蓋中心線的一大半，不能縮在一小段變成連續 S 彎。"""
    v = variant(4)
    xs = [o.map_x for o in v.obstacles]
    assert max(xs) - min(xs) >= 6.0, f"跨距只有 {max(xs)-min(xs):.1f} m"


def _unused_alternation_check():
    for rk, run in ALL_VARIANTS:
        v = variant(run, route=rk)
        sides = []
        for o in v.obstacles:
            # 用最近的中心線點判斷在左邊還是右邊
            best, bs = None, 1e9
            for j in range(0, int(spine_length() * 10) + 1):
                s = j / 10.0
                px, py, hd = spine_point(s)
                d = math.hypot(px - o.map_x, py - o.map_y)
                if d < bs:
                    bs, best = d, (px, py, hd)
            px, py, hd = best
            cross = (-math.sin(hd)) * (o.map_x - px) + math.cos(hd) * (o.map_y - py)
            sides.append(1 if cross > 0 else -1)
        flips = sum(1 for a, b in zip(sides, sides[1:]) if a != b)
        assert flips == len(sides) - 1, f"{rk} run{run} 側邊序列 {sides} 沒有完全交錯"


def test_obstacles_keep_a_minimum_gap_along_the_corridor():
    """★ 兩個障礙沿走廊太近會把路封死。並排那一對除外（刻意靠在一起）。"""
    for rk, run in ALL_VARIANTS:
        obs = variant(run, route=rk).obstacles
        for i in range(len(obs) - 1):
            if obs[i].name.startswith("pair_") and obs[i + 1].name.startswith("pair_"):
                continue
            d = math.hypot(obs[i + 1].map_x - obs[i].map_x,
                           obs[i + 1].map_y - obs[i].map_y)
            assert d >= MIN_OBSTACLE_GAP_M * 0.6, \
                f"{rk} run{run} 第 {i} 與 {i+1} 只差 {d:.2f} m"


def test_obstacles_stay_clear_of_the_route_nodes():
    """★ 障礙壓在車**真的會去**的站點旁，車永遠到不了那個目標。

    2026-09-22 實跑踩到：ped_4_7 落在終點 c25 旁 1.25 m，而抵達判定半徑
    是 1.0 m，整段 FAIL —— 那是設定造成的，不是能力問題。

    只檢查路線上的站。對全部 29 站要求淨空會讓障礙排不下，
    而那些側室站點車不會去。

    ⚠ 站名不要寫死：2026-09-23 路線由 c28↔c25 改成 c28↔c27，
    原本這裡寫 `len(on_route) == 5` 就壞了。跟著 `ROUTE_WAYPOINTS` 走。
    """
    import math

    import ros_graph_spec as S
    from scene_variants import NODE_CLEARANCE_M, route_nodes

    st = S.read_station_nodes(S.ROUTING_STATION_JSON)
    for rk, run in ALL_VARIANTS:
        on_route = route_nodes(st, rk)
        assert set(on_route) == set(S.route(rk).waypoints)
        for o in variant(run, route=rk).obstacles:
            d = min(math.hypot(v[0] - o.map_x, v[1] - o.map_y)
                    for v in on_route.values())
            assert d >= NODE_CLEARANCE_M, \
                f"{rk} run{run} 的 {o.name} 離路線站點只有 {d:.2f} m"


def test_walker_speeds_respect_the_cap():
    from ros_graph_spec import MAX_CHARACTER_SPEED_M_S

    for rk, run in ALL_VARIANTS:
        for w in variant(run, route=rk).walks:
            assert w.speed <= MAX_CHARACTER_SPEED_M_S


def test_walkers_are_distinct_characters():
    """★ 同一個角色被指派兩條路線 → 後者覆蓋前者，人數會少掉而且不報錯。"""
    for rk, run in ALL_VARIANTS:
        names = [w.name for w in variant(run, route=rk).walks]
        assert len(set(names)) == len(names)


def test_standing_and_walking_never_overlap():
    """★ 同一個角色不能既站著又走路 —— 兩個驅動器會搶同一個 transform。"""
    for rk, run in ALL_VARIANTS:
        v = variant(run, route=rk)
        assert not ({p.name for p in v.standing} & {w.name for w in v.walks})


def test_one_standing_person_per_person_obstacle():
    """★ 使用者指定站立障礙要用虛擬人物，不要圓柱。
    每個「人形障礙」都要有一個角色站在同一個位置上。"""
    for rk, run in ALL_VARIANTS:
        v = variant(run, route=rk)
        n_person = sum(1 for o in v.obstacles if o.kind == "person")
        assert len(v.standing) == n_person, \
            f"{rk} run{run} 有 {n_person} 個人形障礙但只擺了 {len(v.standing)} 個角色"
        for p in v.standing:
            assert any(abs(o.map_x - p.map_x) < 1e-6 and abs(o.map_y - p.map_y) < 1e-6
                       for o in v.obstacles), f"{p.name} 沒對到任何障礙位置"


def test_standing_names_are_unique():
    for rk, run in ALL_VARIANTS:
        names = [p.name for p in variant(run, route=rk).standing]
        assert len(set(names)) == len(names)


def test_walk_routes_are_long_enough():
    for rk, run in ALL_VARIANTS:
        for w in variant(run, route=rk).walks:
            L = math.dist(w.waypoints[0], w.waypoints[-1])
            assert L >= 2.5, f"{rk} run{run} 的 {w.name} 單程只有 {L:.1f} m"


def test_spine_point_clamps_at_both_ends():
    """★ 超出兩端不可外插，否則障礙會被放到走廊外面。"""
    a = spine_point(-5.0)
    b = spine_point(spine_length() + 50.0)
    assert (a[0], a[1]) == pytest.approx(CORRIDOR_SPINE[0])
    assert (b[0], b[1]) == pytest.approx(CORRIDOR_SPINE[-1], abs=0.01)


def test_offset_is_perpendicular_to_the_centreline():
    s = 5.0
    px, py, hd = spine_point(s)
    qx, qy = offset_from_spine(s, 1.0)
    dot = (qx - px) * math.cos(hd) + (qy - py) * math.sin(hd)
    assert dot == pytest.approx(0.0, abs=1e-9)


def test_run_index_zero_is_rejected():
    with pytest.raises(ValueError):
        variant(0)


def test_walker_starts_do_not_overlap():
    """★★ 行人起點不可重疊。

    2026-09-23 使用者回報「行人之間會互相穿透」。實測最小間距一律發生在
    `sim_t=0.03s`（第一幀），數值正好等於這裡產生的起點距離
    （run02 0.130 m、run03 0.206 m、run04 0.496 m，門檻是 2×0.30=0.60）。
    不是 ORCA 壞了 —— ORCA 只保證「從不重疊的狀態開始」不會撞，
    解不開一開始就重疊的狀態。
    """
    import math

    from scene_variants import MIN_WALKER_START_GAP_M

    for rk, run in ALL_VARIANTS:
        v = variant(run, route=rk)
        starts = [w.waypoints[0] for w in v.walks]
        for i in range(len(starts)):
            for j in range(i + 1, len(starts)):
                d = math.dist(starts[i], starts[j])
                assert d >= MIN_WALKER_START_GAP_M, (
                    f"{rk} run{run} 的 {v.walks[i].name} 與 {v.walks[j].name} "
                    f"起點只差 {d:.3f} m")


def test_walker_starts_are_clear_of_obstacles():
    """★ 行人起點壓在障礙上，ORCA 的 agent 一開始就在 obstacle 裡面，
    行為未定義（會被擠出去或直接穿過）。"""
    import math

    from scene_variants import MIN_WALKER_START_GAP_M

    for rk, run in ALL_VARIANTS:
        v = variant(run, route=rk)
        for w in v.walks:
            for o in v.obstacles:
                d = math.hypot(w.waypoints[0][0] - o.map_x,
                               w.waypoints[0][1] - o.map_y)
                assert d >= MIN_WALKER_START_GAP_M, (
                    f"{rk} run{run} 的 {w.name} 起點離 {o.name} 只有 {d:.3f} m")


def test_start_gap_check_uses_the_stored_rounded_coordinates():
    """★★ 檢查要用**捨入後**的座標。waypoints 存小數 3 位，拿未捨入的值
    檢查會讓剛好過門檻的那一對存成 0.8494（門檻 0.85）—— 等於檢查沒生效。
    """
    import math

    from scene_variants import MIN_WALKER_START_GAP_M

    v = variant(3)                  # run3 曾是 0.8494 那一組
    starts = [w.waypoints[0] for w in v.walks]
    for p in starts:
        assert all(round(c, 3) == c for c in p), f"{p} 不是存下來的捨入值"
    mn = min(math.dist(a, b) for i, a in enumerate(starts) for b in starts[i + 1:])
    assert mn >= MIN_WALKER_START_GAP_M, mn


def test_obstacles_keep_clear_of_the_goal_and_the_turnaround():
    """★ 障礙不可壓在終點 c36、也不可壓在舊終點 c27 旁（車仍要經過）。

    2026-09-23 擺放範圍改成延伸到終點，改由每個障礙自己的淨空檢查保證
    （原本是「上限離終點 1.8 m」這種全域規則 —— 那條規則只沿中心線算，
    誤以為 c27→c36 擺不下；其實障礙在側邊時離站點的直線距離更大）。
    淨空要算**外接半徑**，道具不是點。
    """
    import ros_graph_spec as S
    from scene_variants import NODE_CLEARANCE_M

    st = S.read_station_nodes(S.ROUTING_STATION_JSON)
    for rk, run in ALL_VARIANTS:
        for o in variant(run, route=rk).obstacles:
            for node in ("c27", "c36"):
                d = math.hypot(st[node][0] - o.map_x, st[node][1] - o.map_y)
                need = NODE_CLEARANCE_M + max(0.0, o.extent_radius - 0.3)
                assert d >= need - 1e-6, (
                    f"{rk} run{run} 的 {o.name} 離 {node} 只有 {d:.2f} m（要 {need:.2f}）")


def _s_along(o):
    best = (1e9, 0.0)
    for j in range(int(spine_length() * 20) + 1):
        sv = j / 20.0
        px, py, _ = spine_point(sv)
        d = math.hypot(px - o.map_x, py - o.map_y)
        if d < best[0]:
            best = (d, sv)
    return best[1]


def test_both_routes_are_defined_and_c27_is_the_main_one():
    """★ 2026-09-23 使用者更正：要的是**兩條路線都錄**（c28↔c27 為主、c28↔c36 為輔），
    不是把終點延長。我先前只做了 c28↔c36，跑了 3 趟才發現。"""
    import ros_graph_spec as S
    assert set(S.ROUTES) == {"c27", "c36"}
    assert (S.route("c27").start, S.route("c27").goal) == ("c28", "c27")
    assert (S.route("c36").start, S.route("c36").goal) == ("c28", "c36")
    assert S.route("c36").waypoints[-2:] == ("c27", "c36")
    assert S.DEFAULT_ROUTE == "c27" and S.ROUTE_ORDER[0] == "c27"
    assert CORRIDOR_SPINE[-1] == pytest.approx((-20.75, 3.17), abs=0.05)


def test_c27_route_puts_nothing_beyond_its_goal():
    """★ c27 路線的車不會開進 c27→c36；那段擺了障礙等於白擺，
    還會吃掉分層抽樣的名額，讓 c28→c27 變稀。"""
    import ros_graph_spec as S
    from scene_variants import route_s_end

    end = route_s_end(S.read_station_nodes(S.ROUTING_STATION_JSON), "c27")
    for run in (1, 2, 3, 4):
        for o in variant(run, route="c27").obstacles:
            assert _s_along(o) < end, f"c27 run{run} 的 {o.name} 在 s={_s_along(o):.2f}"


def test_obstacle_names_are_unique_across_routes():
    """★ 兩條路線的障礙都預先寫進同一份 USD，執行期靠名字開關 —— 撞名的話
    開 c27 的障礙會連帶開到 c36 的。"""
    from scene_variants import all_variant_obstacles
    names = [o.name for o in all_variant_obstacles()]
    assert len(names) == len(set(names))
    assert any("_c27_" in n for n in names) and any("_c36_" in n for n in names)

def test_each_stratum_holds_exactly_one_placement():
    """★★ 分層抽樣：可擺放長度切成 n−1 格，每格恰好一個（並排那一對算一個）。

    2026-09-23 使用者指出分布不平均。舊做法 run1 的 3 個障礙擠在 5.1~8.2 m、
    後面 8.8 m 全空。
    """
    import ros_graph_spec as S
    from scene_variants import (feasible_intervals, measure_slots,
                                route_nodes)

    from scene_variants import OBSTACLE_S_RANGE, SLOT_SPILL, route_s_end

    st = S.read_station_nodes(S.ROUTING_STATION_JSON)

    for rk, run in ALL_VARIANTS:
        iv = feasible_intervals(route_nodes(st, rk), st, 0.25,
                                s_range=(OBSTACLE_S_RANGE[0], route_s_end(st, rk)))

        def t_of(sv):                     # 弧長 → 可擺放長度上的位置
            t = 0.0
            for a, b in iv:
                if sv <= b:
                    return t + max(0.0, sv - a)
                t += b - a
            return t

        v = variant(run, route=rk)
        n_slots = len(v.obstacles) - 1
        slots, _ = measure_slots(iv, n_slots)
        # 並排那一對算一個位置（兩人在彎道上投影到中心線的弧長會差幾公分）
        pair = [_s_along(o) for o in v.obstacles if o.name.startswith("pair_")]
        placements = sorted([_s_along(o) for o in v.obstacles
                             if not o.name.startswith("pair_")]
                            + ([sum(pair) / len(pair)] if pair else []))
        assert len(placements) == n_slots, f"{rk} run{run} 位置數 {placements}"
        for k, sv in enumerate(placements):
            t = t_of(sv)
            t0, t1 = slots[k]
            # 格內擺不下時允許往兩側延伸 SLOT_SPILL 格寬（c27 路線較短才需要）
            spill = SLOT_SPILL * (t1 - t0) + 0.15
            assert t0 - spill <= t <= t1 + spill, (
                f"{rk} run{run} 第 {k} 個（s={sv}，可擺長度 {t:.2f}）不在第 {k} 格 "
                f"[{t0:.2f}, {t1:.2f}]（含延伸 {spill:.2f}）")


def test_obstacles_reach_both_ends_of_the_route():
    """★ 平均的意思是頭尾都要有：最後一個障礙要落在路線可擺放範圍的後段。
    舊版 run1 最後一個在 s=8.2、後面 8.8 m 空白。

    門檻相對於**各自路線**的可擺放範圍（c27 路線較短，終點 s≈17.05）。
    """
    import ros_graph_spec as S
    from scene_variants import (OBSTACLE_S_RANGE, feasible_intervals,
                                route_nodes, route_s_end)

    st = S.read_station_nodes(S.ROUTING_STATION_JSON)
    for rk, run in ALL_VARIANTS:
        iv = feasible_intervals(route_nodes(st, rk), st, 0.25,
                                s_range=(OBSTACLE_S_RANGE[0], route_s_end(st, rk)))
        lo = next(a for a, b in iv if b > a)
        hi = iv[-1][1]
        ss = sorted(_s_along(o) for o in variant(run, route=rk).obstacles)
        span = hi - lo
        assert ss[0] <= lo + 0.4 * span, f"{rk} run{run} 第一個障礙在 s={ss[0]:.1f}"
        assert ss[-1] >= lo + 0.55 * span, (
            f"{rk} run{run} 最後一個障礙在 s={ss[-1]:.1f}，後段空白（可擺到 {hi:.1f}）")

def test_mostly_between_c28_and_c27():
    """★ 使用者要求「主要以 c28→c27 為主」。c27 在 s≈17.0。"""
    total = tail = 0
    for rk, run in ALL_VARIANTS:
        for o in variant(run, route=rk).obstacles:
            total += 1
            tail += _s_along(o) > 17.03
    assert tail / total <= 0.25, f"{tail}/{total} 個在 c27→c36"


def test_side_aware_gap_between_neighbours():
    """★ 同側相鄰 >= 2.2 m（2026-09-22 被 9 個障礙夾死換來的教訓），
    對側相鄰 >= 1.54 m（左右交錯本來就要車蛇行）。並排那一對除外。"""
    from scene_variants import MIN_OBSTACLE_GAP_M, OPPOSITE_SIDE_GAP_RATIO

    for rk, run in ALL_VARIANTS:
        obs = sorted(variant(run, route=rk).obstacles, key=_s_along)
        side = _sides(obs)
        for i in range(len(obs) - 1):
            if obs[i].name.startswith("pair_") and obs[i + 1].name.startswith("pair_"):
                continue
            gap = _s_along(obs[i + 1]) - _s_along(obs[i])
            need = MIN_OBSTACLE_GAP_M * (1.0 if side[i] == side[i + 1]
                                         else OPPOSITE_SIDE_GAP_RATIO)
            assert gap >= need - 0.06, (
                f"{rk} run{run} {obs[i].name}→{obs[i+1].name} 只差 {gap:.2f} m（要 {need:.2f}）")


def test_singles_are_a_balanced_random_mix_of_props_and_people():
    """★ 使用者要求靜態障礙隨機抽成道具或站立行人。用平衡的隨機：
    純隨機在 run1（只有 1 個零星障礙）有一半機率完全沒有道具。"""
    for rk, run in ALL_VARIANTS:
        singles = [o for o in variant(run, route=rk).obstacles if not o.name.startswith("pair_")]
        n_prop = sum(o.kind == "prop" for o in singles)
        assert n_prop == math.ceil(len(singles) / 2), (
            f"{rk} run{run} 零星 {len(singles)} 個裡道具 {n_prop} 個")
        assert all(o.kind in ("prop", "person") for o in singles)


def test_props_come_from_the_lidar_visible_whitelist():
    """★★ 道具一律來自 props.PROPS（每個都實測過會穿過光達那一層）。"""
    from props import MIN_BAND_OVERLAP_M, band_overlap, prop

    for rk, run in ALL_VARIANTS:
        for o in variant(run, route=rk).obstacles:
            if o.kind != "prop":
                continue
            spec = prop(o.asset)
            assert o.height == pytest.approx(spec.height, abs=1e-3)
            assert band_overlap(o.height) >= MIN_BAND_OVERLAP_M


def test_prop_long_side_runs_along_the_corridor():
    """道具長邊要平行走廊（像靠牆擺），不能橫在走廊中間擋住整條路。"""
    from props import prop

    for rk, run in ALL_VARIANTS:
        for o in variant(run, route=rk).obstacles:
            if o.kind != "prop":
                continue
            heading = spine_point(_s_along(o))[2]
            spec = prop(o.asset)
            long_axis = math.radians(o.yaw_deg) + (math.pi / 2 if spec.long_axis_is_y else 0.0)
            assert abs(math.cos(long_axis - heading)) == pytest.approx(1.0, abs=1e-3), o.name


def test_props_keep_clear_of_every_station_by_their_edge():
    """★★ 道具不是點：淨空要用最近的邊算（中心 >= 1.0 + 外接半徑）。
    SM_Cupboard 長 1.84 m，只看中心會讓一端壓到站點。"""
    import ros_graph_spec as S
    from scene_variants import ANY_NODE_CLEARANCE_M

    st = S.read_station_nodes(S.ROUTING_STATION_JSON)
    for rk, run in ALL_VARIANTS:
        for o in variant(run, route=rk).obstacles:
            if o.kind != "prop":
                continue
            d = min(math.hypot(x - o.map_x, y - o.map_y) for x, y, _ in st.values())
            assert d >= ANY_NODE_CLEARANCE_M + o.extent_radius - 1e-6, (
                f"{rk} run{run} 的 {o.name}（{o.asset}）離最近站點 {d:.2f} m")


def test_obstacle_footprints_sit_in_free_space():
    """★★ 每個障礙的 bbox 四角都要落在佔據圖的空地上（離牆 >= 0.10 m）。

    道具靠牆擺、又比人大很多（大盆栽 1.30×1.54 m），側向位移沒算好就會
    插進牆裡 —— 在佔據圖上看不出來，要真的查距離場。
    """
    pytest.importorskip("scipy")
    pytest.importorskip("yaml")
    import os

    import nearest_source as N

    os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    wall = N._wall_distance_field()
    for rk, run in ALL_VARIANTS:
        for o in variant(run, route=rk).obstacles:
            if o.kind == "prop":
                yaw = math.radians(o.yaw_deg)
                hx, hy = o.size_x / 2, o.size_y / 2
                c, s_ = math.cos(yaw), math.sin(yaw)
                corners = [(o.map_x + c * dx - s_ * dy, o.map_y + s_ * dx + c * dy)
                           for dx in (-hx, hx) for dy in (-hy, hy)]
            else:
                corners = [(o.map_x + o.radius * math.cos(a), o.map_y + o.radius * math.sin(a))
                           for a in (0, math.pi / 2, math.pi, 3 * math.pi / 2)]
            for x, y in corners:
                d = wall(x, y)
                assert d is not None and d >= 0.10, (
                    f"{rk} run{run} 的 {o.name} 有一角 ({x:.2f},{y:.2f}) 離牆只有 {d}")


def test_walker_endpoints_avoid_big_props():
    """★ 行人的終點壓在大道具裡的話，ORCA 行人會卡在那裡一直推。"""
    from scene_variants import MIN_WALKER_START_GAP_M

    for rk, run in ALL_VARIANTS:
        v = variant(run, route=rk)
        for w in v.walks:
            for end in w.waypoints:
                for o in v.obstacles:
                    d = math.hypot(end[0] - o.map_x, end[1] - o.map_y)
                    need = MIN_WALKER_START_GAP_M + max(0.0, o.extent_radius - 0.25)
                    assert d >= need - 1e-6, (
                        f"{rk} run{run} 的 {w.name} 端點離 {o.name} 只有 {d:.2f} m")


# ── 2026-09-23：路線感知行人、c36 延伸段障礙、static 停放行人 ──────────────

def _stations():
    return _S_ROUTES.read_station_nodes(_S_ROUTES.ROUTING_STATION_JSON)


def test_walkers_stay_within_their_route():
    """★ c27 路線的行人不能走過 c27（車不去那裡，走了等於白走）。"""
    from scene_variants import route_s_end, spine_coords
    st = _stations()
    for rk, run in ALL_VARIANTS:
        s_end = route_s_end(st, rk)
        for w in variant(run, route=rk).walks:
            for pt in w.waypoints:
                assert spine_coords(pt)[0] <= s_end + 0.05, (rk, run, w.name, pt)


def test_c36_walkers_reach_into_the_extension():
    """★ c36 路線要有行人走進 c27→c36 延伸段，否則延伸段只剩靜態障礙。"""
    from scene_variants import route_s_end, spine_coords
    st = _stations()
    s_main = route_s_end(st, "c27")
    for run in (1, 2, 3, 4):
        far = max(spine_coords(pt)[0] for w in variant(run, route="c36").walks
                  for pt in w.waypoints)
        assert far > s_main, (run, far, s_main)


def test_every_c36_run_has_an_obstacle_in_the_extension():
    """★ 使用者要求：c36 每一趟在 c27→c36 延伸段至少一個障礙。"""
    from scene_variants import route_s_end, spine_coords
    s_main = route_s_end(_stations(), "c27")
    for run in (1, 2, 3, 4):
        ss = [spine_coords((o.map_x, o.map_y))[0]
              for o in variant(run, route="c36").obstacles]
        assert max(ss) >= s_main, (run, ss)


def test_parked_walkers_are_walkers_and_leave_the_passage_open():
    """★ static 停放行人：只能是會走的人，且與**對側**任何東西沿走廊錯開
    >= 1.54 m（兩側同時有東西才會形成窄門，c27 static run4 就是這樣堵死）。"""
    from scene_variants import (MIN_OBSTACLE_GAP_M, OPPOSITE_SIDE_GAP_RATIO,
                                node_clearance_ok, route_nodes, spine_coords)
    st = _stations()
    need = MIN_OBSTACLE_GAP_M * OPPOSITE_SIDE_GAP_RATIO
    for rk, run in ALL_VARIANTS:
        v = variant(run, route=rk)
        assert {p.name for p in v.parked} <= {w.name for w in v.walks}
        things = [(spine_coords((o.map_x, o.map_y)), o.extent_radius, (o.map_x, o.map_y))
                  for o in v.obstacles]
        for p in v.parked:
            (sc, la) = spine_coords((p.map_x, p.map_y))
            assert abs(la) >= 1.05, (rk, run, p.name, la)       # 靠牆，不在中線
            assert node_clearance_ok((p.map_x, p.map_y), route_nodes(st, rk), st, 0.25)
            for (s2, l2), r2, xy in things:
                if (la >= 0) != (l2 >= 0):
                    assert abs(sc - s2) >= need - 1e-6, (rk, run, p.name, s2)
                else:
                    assert math.dist((p.map_x, p.map_y), xy) >= r2 + 0.25 + 0.3 - 1e-6
            things.append(((sc, la), 0.25, (p.map_x, p.map_y)))
