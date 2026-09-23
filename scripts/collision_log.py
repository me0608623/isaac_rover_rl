"""錄影時記下「車身碰到了什麼」——兩個偵測器，各自附心跳。

為什麼需要：「碰撞幀」原本是用光達最近距離 <= 0.45 m 判定，但 PhysX 光達
minRange = 0.5 m（水平最近 0.483 m），讀值在那附近就飽和，比這更近的真實
距離量不到。2026-09-23 對照真值幾何：7 個碰撞幀的真實表面距離是 0.26~0.39 m，
而車體半徑是 0.35 m —— **很可能真的擦到了**，光達讀值看不出來。

兩個偵測器：

1. **重疊查詢（主要）**：每一步拿「外觀車身」大小的盒子問物理引擎
   「這裡面有沒有別人的碰撞體」。
   ⚠ 不能只靠物理碰撞回報：車在物理引擎裡的底盤碰撞體**只是一片
   0.17 × 0.47 m 的薄板**，外觀卻是 0.67 × 0.55 m。外殼擦過障礙、薄板沒碰到，
   物理引擎就沒有紀錄。
   ⚠ 行人的碰撞體對車做了接觸過濾（無限質量的 kinematic 行人會把車彈飛），
   **被過濾的配對不會產生碰撞回報** —— 只看回報的話行人擦撞永遠是 0。
   場景查詢不受過濾影響，所以行人也查得到。

2. **物理碰撞回報（佐證）**：PhysxContactReportAPI，記物理引擎真的算到的接觸
   （例如車真的頂到牆、被推開）。

心跳：偵測器壞掉時最危險的輸出是「0 次擦撞」—— 看起來很乾淨。所以：
  * 重疊查詢：盒子一定包住車自己的碰撞體，每一步都該查到自己。
    查不到的步數比例就是偵測器失效的比例。
  * 碰撞回報：輪子一直貼著地板，一定有「輪子 ↔ 地板」的事件。一個都沒有
    就代表回報沒生效。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

#: 外觀車身（base_link/visuals/mesh_0）在 base_link 座標系的盒子。
#: 實測 x −0.458~+0.212（前 0.21、後 0.46）、y ±0.277、z −0.083~+1.575。
#: 底部從地板往上抬 0.10 m（地板在 base_link z = −0.134）——不抬的話
#: 每一步都會「碰到地板」，而地板跟牆是同一個 Mesh_015，分不出來。
BODY_BOX_CENTRE = (-0.123, 0.0, 0.7705)
BODY_BOX_HALF = (0.335, 0.277, 0.8045)
FLOOR_LIFT_M = 0.10

ROBOT_PREFIX = "/World/charger_rover4_5_0"
HEADER = "t,source,category,object,robot_part,event"

#: 每隔多少模擬秒把 CSV 刷進磁碟、覆寫一次摘要。
#: ⚠ 不能只在結束時寫：批次收尾是 SIGINT、6 秒後 kill -9，Isaac 在那之前
#:   收不完尾 —— 舊錄影的 log 裡「位姿軌跡已寫入」一次都沒出現過，
#:   代表 finally 從來沒跑過。只在 close() 寫的話每一趟都拿不到摘要。
FLUSH_EVERY_S = 1.0

#: 輪子／腳輪貼地的接觸是正常的，不算擦撞（但要數，當心跳）。
_GROUND_PARTS = ("wheel", "caster")


def is_self(path: str) -> bool:
    return bool(path) and path.startswith(ROBOT_PREFIX)


def classify_hit(path: str, walking_names=frozenset()):
    """碰撞體路徑 → ``(類別, 物件名)``；是車自己就回 None。

    類別：道具 / 人形圓柱 / 走動行人 / 站立行人 / 牆 / 地板 / 其他
    """
    if not path or is_self(path):
        return None
    parts = path.strip("/").split("/")
    if path.startswith("/World/SimObstacles/"):
        name = parts[2] if len(parts) > 2 else path
        if name.startswith("prop_"):
            return ("道具", name)
        if name.startswith(("ped_", "pair_")):
            return ("人形圓柱", name)
        return ("其他", name)
    if path.startswith("/World/Characters/"):
        name = parts[2] if len(parts) > 2 else path
        return ("走動行人" if name in walking_names else "站立行人", name)
    if path.startswith("/World/NavFloor"):
        return ("地板", parts[1])
    if path.startswith("/World/Env_0"):
        return ("牆", "Mesh_015")
    return ("其他", path)


def is_ground_contact(robot_part: str, category: str, normal_z: float) -> bool:
    """輪子／腳輪與地板（或建物網格的地面）的正常接觸。"""
    return (any(k in robot_part for k in _GROUND_PARTS)
            and category in ("地板", "牆") and abs(normal_z) > 0.7)


def body_box_world(m):
    """base_link 的世界矩陣（pxr Gf.Matrix4d，row-vector）→
    ``(盒子中心 xyz, 旋轉四元數 xyzw)``。"""
    from pxr import Gf
    c = m.Transform(Gf.Vec3d(*BODY_BOX_CENTRE))
    q = m.ExtractRotationQuat()
    im = q.GetImaginary()
    return (c[0], c[1], c[2]), (im[0], im[1], im[2], q.GetReal())


class EpisodeTracker:
    """把連續幀合併成「一次擦撞」。同一個物件中斷超過 ``gap_s`` 秒才算新的一次。"""

    def __init__(self, gap_s: float = 0.2):
        self.gap_s = gap_s
        self._open: dict = {}          # (類別, 物件) → [開始, 最後一次]
        self.episodes: list = []       # (類別, 物件, 開始, 結束)

    def hit(self, t: float, key) -> None:
        cur = self._open.get(key)
        if cur is not None and t - cur[1] <= self.gap_s:
            cur[1] = t
            return
        if cur is not None:
            self.episodes.append((*key, cur[0], cur[1]))
        self._open[key] = [t, t]

    def snapshot(self) -> list:
        """目前為止的所有擦撞（含還沒結束的），**不改變狀態**。"""
        out = list(self.episodes) + [(*k, a, b) for k, (a, b) in self._open.items()]
        return sorted(out, key=lambda e: e[2])

    def close(self) -> list:
        for key, (a, b) in self._open.items():
            self.episodes.append((*key, a, b))
        self._open.clear()
        self.episodes.sort(key=lambda e: e[2])
        return self.episodes


def merged_events(episodes, gap_s: float = 0.2) -> list:
    """把時間上重疊（或相隔 <= gap_s）的擦撞合併成「一次事件」。

    ⚠ 站立行人腳邊疊了一根隱形碰撞圓柱，車碰到那個人時重疊查詢會同時回報
    「人形圓柱」與「站立行人」—— 不合併的話一次擦撞算成兩次。
    """
    iv = sorted((a, b) for _c, _o, a, b in episodes)
    out: list = []
    for a, b in iv:
        if out and a - out[-1][1] <= gap_s:
            out[-1][1] = max(out[-1][1], b)
        else:
            out.append([a, b])
    return [tuple(x) for x in out]


def summarise(episodes, overlap_steps: int, overlap_self_steps: int,
              ground_contacts: int) -> dict:
    """給 run.json 的摘要。偵測器有沒有在工作，要跟結果一起報。

    ``episodes`` 是**合併後的事件數**；``by_category`` 是各類別原始次數
    （同一次事件可能同時出現在兩個類別）。
    """
    by_cat: dict = {}
    for cat, _obj, a, b in episodes:
        d = by_cat.setdefault(cat, {"episodes": 0, "seconds": 0.0})
        d["episodes"] += 1
        d["seconds"] = round(d["seconds"] + (b - a), 3)
    frac = overlap_self_steps / overlap_steps if overlap_steps else 0.0
    ev = merged_events(episodes)
    return {
        "episodes": len(ev),
        "seconds": round(sum(b - a for a, b in ev), 3),
        "by_category": by_cat,
        "detector": {
            "overlap_steps": overlap_steps,
            "overlap_self_seen_ratio": round(frac, 4),
            "overlap_ok": overlap_steps > 0 and frac >= 0.99,
            "ground_contacts": ground_contacts,
            "contact_report_ok": ground_contacts > 0,
        },
    }


def apply_contact_report_api(stage) -> int:
    """在車的每個剛體套 PhysxContactReportAPI（門檻 0 = 全部回報）。回傳套了幾個。

    ⚠ 必須在 ``sim.play()`` **之前**呼叫：物理引擎在 play 時解析 stage，
    之後才套的 API 不一定被讀到 —— 那樣整批錄影的碰撞回報都是空的
    （心跳會抓到，但那等於白錄一批）。
    """
    from pxr import PhysxSchema, Sdf, UsdPhysics

    n = 0
    for p in stage.Traverse():
        if str(p.GetPath()).startswith(ROBOT_PREFIX) and p.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(p)
            p.CreateAttribute("physxContactReport:threshold",
                              Sdf.ValueTypeNames.Float).Set(0.0)
            n += 1
    return n


class CollisionLogger:
    """Isaac 執行期用。每一步呼叫 ``step()``，結束呼叫 ``close()``。

    碰撞回報的 API 要先在 play 之前用 ``apply_contact_report_api`` 套好。
    """

    def __init__(self, stage, csv_path, walking_names=frozenset(), gap_s=0.2):
        self._walking = frozenset(walking_names)
        self._fp = open(csv_path, "w")
        self._fp.write(HEADER + "\n")
        self._csv_path = Path(csv_path)
        self._tracker = EpisodeTracker(gap_s)
        self._t = 0.0
        self._last_flush = -1e9
        self.overlap_steps = 0
        self.overlap_self_steps = 0
        self.ground_contacts = 0

        from omni.physx import get_physx_simulation_interface
        self._sub = get_physx_simulation_interface().subscribe_contact_report_events(
            self._on_contact)
        print(f"[collision_log] 訂閱碰撞回報；重疊查詢盒半邊長 {BODY_BOX_HALF}"
              f" → {csv_path}")

    # ── 物理碰撞回報 ──────────────────────────────────────────────────
    def _on_contact(self, headers, data):
        from omni.physx.bindings._physx import ContactEventType
        from pxr import PhysicsSchemaTools as PST
        for h in headers:
            c0 = str(PST.intToSdfPath(h.collider0))
            c1 = str(PST.intToSdfPath(h.collider1))
            if is_self(c0) == is_self(c1):
                continue                          # 車自己碰自己、或與車無關
            robot_part, other = (c0, c1) if is_self(c0) else (c1, c0)
            cls = classify_hit(other, self._walking)
            if cls is None:
                continue
            nz = 0.0
            if h.num_contact_data:
                nz = float(data[h.contact_data_offset].normal[2])
            if is_ground_contact(robot_part, cls[0], nz):
                self.ground_contacts += 1         # 心跳：正常貼地
                continue
            ev = {ContactEventType.CONTACT_FOUND: "found",
                  ContactEventType.CONTACT_PERSISTS: "persist",
                  ContactEventType.CONTACT_LOST: "lost"}.get(h.type, str(h.type))
            part = robot_part[len(ROBOT_PREFIX):]
            self._fp.write(f"{self._t:.4f},contact,{cls[0]},{cls[1]},{part},{ev}\n")
            if ev != "lost":
                self._tracker.hit(self._t, (cls[0], cls[1]))

    # ── 重疊查詢 ──────────────────────────────────────────────────────
    def step(self, t: float, base_link_world) -> None:
        import carb
        from omni.physx import get_physx_scene_query_interface

        self._t = t
        (cx, cy, cz), (qx, qy, qz, qw) = body_box_world(base_link_world)
        hits = []

        def report(hit):
            hits.append(hit.collision)
            return True

        get_physx_scene_query_interface().overlap_box(
            carb.Float3(*BODY_BOX_HALF), carb.Float3(cx, cy, cz),
            carb.Float4(qx, qy, qz, qw), report, False)
        self.overlap_steps += 1
        if any(is_self(h) for h in hits):
            self.overlap_self_steps += 1          # 心跳：一定包住自己
        seen = set()
        for path in hits:
            cls = classify_hit(path, self._walking)
            if cls is None or cls in seen or cls[0] == "地板":
                continue
            seen.add(cls)
            self._fp.write(f"{t:.4f},overlap,{cls[0]},{cls[1]},body,in\n")
            self._tracker.hit(t, cls)
        if t - self._last_flush >= FLUSH_EVERY_S:
            self._last_flush = t
            self._fp.flush()
            self._write_summary(self._tracker.snapshot(), final=False)

    def _write_summary(self, episodes, final: bool) -> dict:
        s = summarise(episodes, self.overlap_steps, self.overlap_self_steps,
                      self.ground_contacts)
        s["final"] = final                  # False = 被 kill 前最後一次的定期存檔
        s["sim_time"] = round(self._t, 3)
        out = self._csv_path.with_name("collisions_summary.json")
        tmp = out.with_suffix(".tmp")
        tmp.write_text(json.dumps(s, ensure_ascii=False, indent=1))
        tmp.replace(out)                    # 原子替換：kill 在寫一半時不會留下壞檔
        return s

    def close(self) -> dict:
        self._sub = None
        self._fp.close()
        s = self._write_summary(self._tracker.close(), final=True)
        d = s["detector"]
        print(f"[collision_log] 擦撞 {s['episodes']} 次 {s['by_category']}　"
              f"重疊查詢看到自己 {d['overlap_self_seen_ratio']:.1%}"
              f"{'' if d['overlap_ok'] else ' ⚠ 偵測器沒在工作'}　"
              f"輪子貼地事件 {d['ground_contacts']}"
              f"{'' if d['contact_report_ok'] else ' ⚠ 碰撞回報沒生效'}")
        return s
