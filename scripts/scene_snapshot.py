"""把「這一趟場上實際有什麼」寫成 scene.json，事後分析一律讀它。

為什麼需要：分析工具原本用 ``scene_variants.variant(i)`` 取場景，但那是
**現在的程式碼**算出來的。2026-09-23 障礙擺法改成分層抽樣、加入道具之後，
同一個 run_index 在新舊程式碼裡位置完全不同 —— 拿新程式分析舊錄影會把
障礙放錯位置、算出錯的距離，而且**不會報錯**。

另外執行期還會停用角色（太靠近車、站在 routing 點上、擠在一起、障礙關閉），
計畫表也看不到。所以要在錄影當下，把**設定完成後**真的在場的東西存下來。

⚠⚠ 第二遍回放（replay_render）也**一律照這份快照擺場景**，不再自己重算。
2026-09-23 發現兩遍各寫一份規則已經走樣：第一遍改成「dynamic 停用站立人物」、
「static 停用站在導航點旁的人」，回放那邊沒跟著改 —— dynamic 與 static 的影片裡
會出現導航時根本不在場的人。v2 多記走動行人的路線與站立人物的朝向，
回放拿到快照就能完整重建，不需要 variant()。
"""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

SNAPSHOT_NAME = "scene.json"
SNAPSHOT_VERSION = 2
#: 讀得懂的版本。v1（沒有 walks / yaw / route）只夠分析用，不夠回放。
READABLE_VERSIONS = (1, 2)


def build_snapshot(scenario: str, run_index: int, obstacles, characters,
                   walks=(), standing_yaw=None, route: str = "") -> dict:
    """``obstacles``：啟用中的 Obstacle；
    ``characters``：``(名字, map_x, map_y, 會不會走)``；
    ``walks``：會走的人的 CharacterWalk（回放建步態要用）；
    ``standing_yaw``：``{名字: map yaw}``，被 place_standing 擺位的站立人物朝向。"""
    yaw = dict(standing_yaw or {})
    return {
        "version": SNAPSHOT_VERSION,
        "scenario": scenario,
        "run_index": int(run_index),
        "route": route,
        "obstacles": [asdict(o) for o in obstacles],
        "characters": [
            {"name": n, "map_x": round(float(x), 4), "map_y": round(float(y), 4),
             "walking": bool(w), "placed_yaw": yaw.get(n)}
            for n, x, y, w in characters
        ],
        "walks": [asdict(w) for w in walks],
    }


def write_snapshot(run_dir, snap: dict) -> Path:
    p = Path(run_dir) / SNAPSHOT_NAME
    p.write_text(json.dumps(snap, ensure_ascii=False, indent=1))
    return p


def read_snapshot(run_dir):
    """讀 scene.json；沒有就回 None（舊錄影沒有，呼叫端要自己決定怎麼辦）。"""
    p = Path(run_dir) / SNAPSHOT_NAME
    if not p.exists():
        return None
    snap = json.loads(p.read_text())
    if snap.get("version") not in READABLE_VERSIONS:
        raise ValueError(f"{p} 版本 {snap.get('version')}，只看得懂 {READABLE_VERSIONS}")
    return snap


def obstacles_of(snap):
    """snapshot → Obstacle 物件串列。"""
    from ros_graph_spec import Obstacle
    return [Obstacle(**o) for o in snap["obstacles"]]


def still_bodies(snap):
    """不會走的角色（站立人物 + static 情境停在原位的人）的 map 位置。"""
    return [(c["map_x"], c["map_y"]) for c in snap["characters"] if not c["walking"]]


def walks_of(snap):
    """snapshot → CharacterWalk 串列（v2 才有）。"""
    from ros_graph_spec import CharacterWalk
    if snap.get("version", 1) < 2:
        raise ValueError("v1 快照沒有記走動行人的路線，不能拿來回放")
    out = []
    for w in snap["walks"]:
        d = dict(w)
        d["waypoints"] = tuple(tuple(p) for p in d["waypoints"])
        out.append(CharacterWalk(**d))
    return out


def placed_standing(snap):
    """被 place_standing 擺位的站立人物 → ``[(名字, map_x, map_y, yaw)]``。

    static 情境裡不走的行人也在這裡：2026-09-23 起他們被停放到變體指定的
    安全位置（``SceneVariant.parked``），不再留在 USD 原位。
    """
    return [(c["name"], c["map_x"], c["map_y"], c["placed_yaw"])
            for c in snap["characters"]
            if not c["walking"] and c.get("placed_yaw") is not None]
