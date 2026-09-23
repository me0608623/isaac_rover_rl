"""場景變體的測試。"""

from __future__ import annotations

import math

import pytest

from scene_variants import (CORRIDOR_SPINE, MIN_OBSTACLE_GAP_M, OBSTACLE_COUNTS,
                            WALKER_COUNTS, offset_from_spine, spine_length,
                            spine_point, variant)


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
    for run in (1, 2, 3, 4):
        pair = [o for o in variant(run).obstacles if o.name.startswith("pair_")]
        assert len(pair) == 2, f"run{run} 有 {len(pair)} 個並排障礙"
        d = math.hypot(pair[0].map_x - pair[1].map_x, pair[0].map_y - pair[1].map_y)
        assert 0.4 <= d <= 0.9, f"run{run} 兩人相距 {d:.2f} m，不像肩並肩"
        assert _sides(pair)[0] == _sides(pair)[1], f"run{run} 那一對不在同一側"
        assert all(o.kind == "person" for o in pair), "並排的必須是人，不是推車"


def test_single_obstacles_alternate_sides():
    """★ 零星障礙要左右交錯（並排那一對不算，它刻意同側）。"""
    for run in (1, 2, 3, 4):
        v = variant(run)
        singles = [o for o in v.obstacles if not o.name.startswith("pair_")]
        if len(singles) < 2:
            continue
        sides = _sides(singles)
        flips = sum(1 for a, b in zip(sides, sides[1:]) if a != b)
        assert flips == len(sides) - 1, f"run{run} 零星障礙側邊 {sides} 沒交錯"


def test_obstacles_are_spread_over_the_whole_corridor():
    """★ 使用者指出「走廊很長，應該可以平均放置」——
    最難那趟的障礙跨距要涵蓋中心線的一大半，不能縮在一小段變成連續 S 彎。"""
    v = variant(4)
    xs = [o.map_x for o in v.obstacles]
    assert max(xs) - min(xs) >= 6.0, f"跨距只有 {max(xs)-min(xs):.1f} m"


def _unused_alternation_check():
    for run in (1, 2, 3, 4):
        v = variant(run)
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
        assert flips == len(sides) - 1, f"run{run} 側邊序列 {sides} 沒有完全交錯"


def test_obstacles_keep_a_minimum_gap_along_the_corridor():
    """★ 兩個障礙沿走廊太近會把路封死。並排那一對除外（刻意靠在一起）。"""
    for run in (1, 2, 3, 4):
        obs = variant(run).obstacles
        for i in range(len(obs) - 1):
            if obs[i].name.startswith("pair_") and obs[i + 1].name.startswith("pair_"):
                continue
            d = math.hypot(obs[i + 1].map_x - obs[i].map_x,
                           obs[i + 1].map_y - obs[i].map_y)
            assert d >= MIN_OBSTACLE_GAP_M * 0.6, \
                f"run{run} 第 {i} 與 {i+1} 只差 {d:.2f} m"


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

    on_route = route_nodes(S.read_station_nodes(S.ROUTING_STATION_JSON))
    assert set(on_route) == set(S.ROUTE_WAYPOINTS)
    for run in (1, 2, 3, 4):
        for o in variant(run).obstacles:
            d = min(math.hypot(v[0] - o.map_x, v[1] - o.map_y)
                    for v in on_route.values())
            assert d >= NODE_CLEARANCE_M, \
                f"run{run} 的 {o.name} 離路線站點只有 {d:.2f} m"


def test_walker_speeds_respect_the_cap():
    from ros_graph_spec import MAX_CHARACTER_SPEED_M_S

    for run in (1, 2, 3, 4):
        for w in variant(run).walks:
            assert w.speed <= MAX_CHARACTER_SPEED_M_S


def test_walkers_are_distinct_characters():
    """★ 同一個角色被指派兩條路線 → 後者覆蓋前者，人數會少掉而且不報錯。"""
    for run in (1, 2, 3, 4):
        names = [w.name for w in variant(run).walks]
        assert len(set(names)) == len(names)


def test_standing_and_walking_never_overlap():
    """★ 同一個角色不能既站著又走路 —— 兩個驅動器會搶同一個 transform。"""
    for run in (1, 2, 3, 4):
        v = variant(run)
        assert not ({p.name for p in v.standing} & {w.name for w in v.walks})


def test_one_standing_person_per_person_obstacle():
    """★ 使用者指定站立障礙要用虛擬人物，不要圓柱。
    每個「人形障礙」都要有一個角色站在同一個位置上。"""
    for run in (1, 2, 3, 4):
        v = variant(run)
        n_person = sum(1 for o in v.obstacles if o.kind == "person")
        assert len(v.standing) == n_person, \
            f"run{run} 有 {n_person} 個人形障礙但只擺了 {len(v.standing)} 個角色"
        for p in v.standing:
            assert any(abs(o.map_x - p.map_x) < 1e-6 and abs(o.map_y - p.map_y) < 1e-6
                       for o in v.obstacles), f"{p.name} 沒對到任何障礙位置"


def test_standing_names_are_unique():
    for run in (1, 2, 3, 4):
        names = [p.name for p in variant(run).standing]
        assert len(set(names)) == len(names)


def test_walk_routes_are_long_enough():
    for run in (1, 2, 3, 4):
        for w in variant(run).walks:
            L = math.dist(w.waypoints[0], w.waypoints[-1])
            assert L >= 2.5, f"run{run} 的 {w.name} 單程只有 {L:.1f} m"


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

    for run in (1, 2, 3, 4):
        v = variant(run)
        starts = [w.waypoints[0] for w in v.walks]
        for i in range(len(starts)):
            for j in range(i + 1, len(starts)):
                d = math.dist(starts[i], starts[j])
                assert d >= MIN_WALKER_START_GAP_M, (
                    f"run{run} 的 {v.walks[i].name} 與 {v.walks[j].name} "
                    f"起點只差 {d:.3f} m")


def test_walker_starts_are_clear_of_obstacles():
    """★ 行人起點壓在障礙上，ORCA 的 agent 一開始就在 obstacle 裡面，
    行為未定義（會被擠出去或直接穿過）。"""
    import math

    from scene_variants import MIN_WALKER_START_GAP_M

    for run in (1, 2, 3, 4):
        v = variant(run)
        for w in v.walks:
            for o in v.obstacles:
                d = math.hypot(w.waypoints[0][0] - o.map_x,
                               w.waypoints[0][1] - o.map_y)
                assert d >= MIN_WALKER_START_GAP_M, (
                    f"run{run} 的 {w.name} 起點離 {o.name} 只有 {d:.3f} m")


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


def test_obstacles_keep_clear_of_the_far_end_goal():
    """★ 終點由側邊的 c25 改成 spine 盡頭的 c27 之後，障礙上限若還是 s=16，
    離終點只剩 1.03 m。OBSTACLE_S_RANGE 要留出 NODE_CLEARANCE_M。"""
    from scene_variants import (NODE_CLEARANCE_M, OBSTACLE_S_RANGE,
                                spine_length)

    assert spine_length() - OBSTACLE_S_RANGE[1] >= NODE_CLEARANCE_M
