"""把「這一趟場上實際有什麼」寫成 scene.json，事後分析一律讀它。

為什麼需要：分析工具原本用 ``scene_variants.variant(i)`` 取場景，但那是
**現在的程式碼**算出來的。2026-09-23 障礙擺法改成分層抽樣、加入道具之後，
同一個 run_index 在新舊程式碼裡位置完全不同 —— 拿新程式分析舊錄影會把
障礙放錯位置、算出錯的距離，而且**不會報錯**。

另外執行期還會停用角色（太靠近車、站在 routing 點上、擠在一起、障礙關閉），
計畫表也看不到。所以要在錄影當下，把**設定完成後**真的在場的東西存下來。
"""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

SNAPSHOT_NAME = "scene.json"
SNAPSHOT_VERSION = 1


def build_snapshot(scenario: str, run_index: int, obstacles, characters) -> dict:
    """``obstacles``：啟用中的 Obstacle；``characters``：``(名字, map_x, map_y, 會不會走)``。"""
    return {
        "version": SNAPSHOT_VERSION,
        "scenario": scenario,
        "run_index": int(run_index),
        "obstacles": [asdict(o) for o in obstacles],
        "characters": [
            {"name": n, "map_x": round(float(x), 4), "map_y": round(float(y), 4),
             "walking": bool(w)}
            for n, x, y, w in characters
        ],
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
    if snap.get("version") != SNAPSHOT_VERSION:
        raise ValueError(f"{p} 版本 {snap.get('version')}，只看得懂 {SNAPSHOT_VERSION}")
    return snap


def obstacles_of(snap):
    """snapshot → Obstacle 物件串列。"""
    from ros_graph_spec import Obstacle
    return [Obstacle(**o) for o in snap["obstacles"]]


def still_bodies(snap):
    """不會走的角色（站立人物 + static 情境停在原位的人）的 map 位置。"""
    return [(c["map_x"], c["map_y"]) for c in snap["characters"] if not c["walking"]]
