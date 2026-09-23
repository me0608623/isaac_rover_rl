"""錄影時記下「車身碰到了什麼」—— 外觀車身盒子的重疊查詢，附心跳與正向對照。

為什麼需要：「碰撞幀」原本是用光達最近距離 <= 0.45 m 判定，但 PhysX 光達
minRange = 0.5 m（水平最近 0.483 m），讀值在那附近就飽和，比這更近的真實
距離量不到。2026-09-23 對照真值幾何：7 個碰撞幀的真實表面距離是 0.26~0.39 m，
而車體半徑是 0.35 m —— **很可能真的擦到了**，光達讀值看不出來。

做法：每一步拿「外觀車身」大小的盒子問物理引擎「這裡面有沒有別人的碰撞體」
（場景查詢 overlap_box）。

⚠ 為什麼**不用**物理碰撞回報（PhysxContactReportAPI）—— 2026-09-23 實測後拿掉：
  * 車停在起點不動 6 秒，碰撞回報記了「撞牆 5.97 秒」：底盤碰撞體是一片
    往下伸到**地板下 0.38 m** 的薄板，一直插在建物網格 Mesh_015 裡，而地板與牆
    是同一個網格，分不出來。整批跑下去每一趟都會從頭到尾「撞牆」。
    同一段時間重疊查詢一筆假的都沒有。
  * 6 秒冒出 10,025 行 PhysX 警告（getMaterialFromInternalFaceIndex
    received 0xFFFFffff）；改之前的錄影是 0 行。
  * 底盤碰撞體只是 0.17 × 0.47 m 的薄板（外觀 0.67 × 0.55 m），外殼擦過、
    薄板沒碰到就沒有接觸；行人又對車做了接觸過濾，永遠不會回報。
  * 它的 callback 是 C++ 呼叫的，裡面丟出例外會讓整個 Isaac 當掉（文件寫的
    ``CONTACT_PERSISTS`` 在這一版叫 ``CONTACT_PERSIST``，第一個事件就當掉）。

偵測器壞掉時最危險的輸出是「0 次擦撞」—— 看起來很乾淨。所以要兩種檢查：
  * **心跳**：盒子一定包住車自己的碰撞體，每一步都該查到自己。
  * **正向對照**：開跑後故意在一個已知的靜態障礙、和一個走動行人的位置各查一次，
    必須查得到。只看心跳的話，「查得到自己、查不到別人」（例如別人的碰撞體
    在查詢不到的群組裡）會被當成正常。
"""

from __future__ import annotations

import json
from pathlib import Path

#: 外觀車身（base_link/visuals/mesh_0）在 base_link 座標系的盒子。
#: 實測 x −0.458~+0.212（前 0.21、後 0.46）、y ±0.277、z −0.083~+1.575。
#: 底部從地板往上抬 0.10 m（地板在 base_link z = −0.134）——不抬的話
#: 每一步都會「碰到地板」，而地板跟牆是同一個 Mesh_015，分不出來。
BODY_BOX_CENTRE = (-0.123, 0.0, 0.7705)
BODY_BOX_HALF = (0.335, 0.277, 0.8045)
FLOOR_LIFT_M = 0.10

ROBOT_PREFIX = "/World/charger_rover4_5_0"
HEADER = "t,category,object"

#: 每隔多少模擬秒把 CSV 刷進磁碟、覆寫一次摘要。
#: ⚠ 不能只在結束時寫：批次收尾是 SIGINT、6 秒後 kill -9，Isaac 在那之前
#:   收不完尾 —— 舊錄影的 log 裡「位姿軌跡已寫入」一次都沒出現過，
#:   代表 finally 從來沒跑過。只在 close() 寫的話每一趟都拿不到摘要。
FLUSH_EVERY_S = 1.0

#: 開跑後第幾步做正向對照。不能在第 0 步：行人的逐部位碰撞體要等部位驅動器
#: 跑過一次才會擺到身上。
CONTROL_AT_STEP = 10


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
              controls=None) -> dict:
    """給 run.json 的摘要。偵測器有沒有在工作，要跟結果一起報。

    ``controls``：正向對照 ``{"障礙": True/False/None, "行人": ...}``；
    None = 場上沒有那種東西、無從對照（例如 dynamic 沒有靜態障礙）。

    ``episodes`` 是**合併後的事件數**；``by_category`` 是各類別原始次數
    （同一次事件可能同時出現在兩個類別）。
    """
    controls = dict(controls or {})
    by_cat: dict = {}
    for cat, _obj, a, b in episodes:
        d = by_cat.setdefault(cat, {"episodes": 0, "seconds": 0.0})
        d["episodes"] += 1
        d["seconds"] = round(d["seconds"] + (b - a), 3)
    frac = overlap_self_steps / overlap_steps if overlap_steps else 0.0
    ev = merged_events(episodes)
    controls_ok = all(v is not False for v in controls.values())
    return {
        "episodes": len(ev),
        "seconds": round(sum(b - a for a, b in ev), 3),
        "by_category": by_cat,
        "detector": {
            "overlap_steps": overlap_steps,
            "overlap_self_seen_ratio": round(frac, 4),
            "positive_controls": controls,
            "overlap_ok": overlap_steps > 0 and frac >= 0.99 and controls_ok,
        },
    }


class CollisionLogger:
    """Isaac 執行期用。每一步呼叫 ``step()``，結束呼叫 ``close()``。"""

    def __init__(self, stage, csv_path, walking_names=frozenset(), gap_s=0.2):
        self._stage = stage
        self._walking = frozenset(walking_names)
        self._fp = open(csv_path, "w")
        self._fp.write(HEADER + "\n")
        self._csv_path = Path(csv_path)
        self._tracker = EpisodeTracker(gap_s)
        self._t = 0.0
        self._last_flush = -1e9
        self.overlap_steps = 0
        self.overlap_self_steps = 0
        self.controls: dict = {}
        print(f"[collision_log] 重疊查詢盒半邊長 {BODY_BOX_HALF} → {csv_path}")

    # ── 查詢 ──────────────────────────────────────────────────────────
    @staticmethod
    def _query(centre, half, quat_xyzw=(0.0, 0.0, 0.0, 1.0)) -> list:
        import carb
        from omni.physx import get_physx_scene_query_interface

        hits = []

        def report(hit):
            hits.append(hit.collision)
            return True

        get_physx_scene_query_interface().overlap_box(
            carb.Float3(*half), carb.Float3(*centre), carb.Float4(*quat_xyzw),
            report, False)
        return hits

    def _positive_controls(self) -> None:
        """在一個已知的靜態障礙、一個走動行人身上各查一次，必須查得到。"""
        from pxr import UsdGeom

        cache = UsdGeom.XformCache()
        # 靜態障礙：第一個啟用中的；道具查它的 Collider，圓柱查它自己
        obs = None
        root = self._stage.GetPrimAtPath("/World/SimObstacles")
        if root and root.IsValid():
            for c in sorted(root.GetChildren(), key=lambda p: p.GetName()):
                if c.IsActive():
                    col = self._stage.GetPrimAtPath(f"{c.GetPath()}/Collider")
                    obs = (c.GetName(), col if col and col.IsValid() else c)
                    break
        if obs is None:
            self.controls["障礙"] = None
        else:
            t = cache.GetLocalToWorldTransform(obs[1]).ExtractTranslation()
            hits = self._query((t[0], t[1], t[2]), (0.1, 0.1, 0.1))
            self.controls["障礙"] = any(f"/SimObstacles/{obs[0]}" in h for h in hits)
        # 走動行人：第一個會走的；查他軀幹那一段
        ped = None
        croot = self._stage.GetPrimAtPath("/World/Characters")
        if croot and croot.IsValid():
            for c in sorted(croot.GetChildren(), key=lambda p: p.GetName()):
                if c.IsActive() and c.GetName() in self._walking:
                    ped = c
                    break
        if ped is None:
            self.controls["行人"] = None
        else:
            t = cache.GetLocalToWorldTransform(ped).ExtractTranslation()
            hits = self._query((t[0], t[1], t[2] + 1.0), (0.25, 0.25, 0.4))
            self.controls["行人"] = any(
                h.startswith(f"/World/Characters/{ped.GetName()}/") for h in hits)
        bad = [k for k, v in self.controls.items() if v is False]
        print(f"[collision_log] 正向對照 {self.controls}"
              + (f"　⚠ 查不到{'、'.join(bad)} —— 偵測器對它們是瞎的" if bad else ""))

    def step(self, t: float, base_link_world) -> None:
        self._t = t
        (cx, cy, cz), q = body_box_world(base_link_world)
        hits = self._query((cx, cy, cz), BODY_BOX_HALF, q)
        self.overlap_steps += 1
        if any(is_self(h) for h in hits):
            self.overlap_self_steps += 1          # 心跳：一定包住自己
        if self.overlap_steps == CONTROL_AT_STEP:
            try:
                self._positive_controls()
            except Exception as e:                # 對照本身壞了也要說
                self.controls["錯誤"] = False
                print(f"[collision_log] ⚠ 正向對照失敗：{e!r}")
        seen = set()
        for path in hits:
            cls = classify_hit(path, self._walking)
            if cls is None or cls in seen or cls[0] == "地板":
                continue
            seen.add(cls)
            self._fp.write(f"{t:.4f},{cls[0]},{cls[1]}\n")
            self._tracker.hit(t, cls)
        if t - self._last_flush >= FLUSH_EVERY_S:
            self._last_flush = t
            self._fp.flush()
            self._write_summary(self._tracker.snapshot(), final=False)

    def _write_summary(self, episodes, final: bool) -> dict:
        s = summarise(episodes, self.overlap_steps, self.overlap_self_steps,
                      self.controls)
        s["final"] = final                  # False = 被 kill 前最後一次的定期存檔
        s["sim_time"] = round(self._t, 3)
        out = self._csv_path.with_name("collisions_summary.json")
        tmp = out.with_suffix(".tmp")
        tmp.write_text(json.dumps(s, ensure_ascii=False, indent=1))
        tmp.replace(out)                    # 原子替換：kill 在寫一半時不會留下壞檔
        return s

    def close(self) -> dict:
        self._fp.close()
        s = self._write_summary(self._tracker.close(), final=True)
        d = s["detector"]
        print(f"[collision_log] 擦撞 {s['episodes']} 次 {s['by_category']}　"
              f"查詢看到自己 {d['overlap_self_seen_ratio']:.1%}　"
              f"正向對照 {d['positive_controls']}"
              f"{'' if d['overlap_ok'] else ' ⚠ 偵測器沒在工作'}")
        return s
