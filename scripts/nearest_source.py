#!/usr/bin/env python3
"""把「最近障礙距離」分解成：走動行人 / 站立行人 / 箱型障礙 / 牆。

為什麼要做：`monitor_navigation` 報的「最近障礙」是 `/velodyne_points`
**整片點雲**的最小水平距離（感測器座標、只留 |z|≤0.5，等於離地
0.93~1.93 m 那一圈）。那一圈裡同時有行人、靜態障礙、**還有牆**。

走廊本來就窄，貼著牆走是正常的，不該算成危險。所以「碰撞幀」
（最近距離 ≤0.45 m 的取樣數）裡若有一部分其實是牆，那個指標就不能
直接當論文的碰撞證據。2026-09-23 發現有數趟的最近距離**恰好** 0.45 ——
必須查清楚那是誰。

⚠ 這支程式自己會**驗證**：預測的最近距離要跟 nav CSV 裡實際量到的
`min_range_m` 對得上。對不上就不要相信分解結果（殘差會印出來）。
上一次「取最後 4 幀平均」的教訓就是沒有做這種對照。

各情境實際在場上的東西（照 run_isaac_sim 的實際行為，不是 README）：

    情境      圓柱/箱型障礙   站立人物            走動人物
    static    啟用            在障礙位置          **站在 USD 原位**
    dynamic   全部關閉        在障礙位置(仍在場)  ORCA
    mixed     啟用            在障礙位置          ORCA

  ⚠ dynamic 的「靜態障礙 0/18 啟用」只關掉圓柱，`place_standing` 還是
    把站立人物擺到那些位置 —— 人還在，光達打得到。
  ⚠ static 的走動人物不是消失，而是**停在 USD 原始位置**；其中
    Character_10~13、19 的原位剛好就在走廊中線上。

用法：
    PYTHONPATH= .venv/bin/python scripts/nearest_source.py recordings
"""

from __future__ import annotations

import bisect
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from run_layout import run_dirs

#: 走動行人在光達帶裡的等效半徑（m）。逐部位碰撞體，肩寬實測 0.45 m。
PED_RADIUS_M = 0.22

#: 站立人物的等效半徑（m）。static/mixed 底下人物腳邊還疊一根
#: 不可見圓柱（r=0.25）當碰撞體，取兩者較大值。
STANDING_RADIUS_M = 0.25

#: 判定「太靠近」的門檻，與 monitor_navigation 的碰撞幀定義一致。
NEAR_THRESHOLD_M = 0.45

#: 分類標籤。順序 = 報表欄位順序。
CATEGORIES = ("走動行人", "站立行人", "箱型障礙", "牆")

#: 實測比預測近這麼多以上（m），就**不歸給任何一類**，標成「來源不明」。
#:
#: 2026-09-23 抓到的實例：sa4r3_mixed_run04 有 4 個取樣點實測 0.447 m，
#: 但場上最近的已知物體在 1.75 m 外。把那片回波用 bag 的 TF 搬到 map
#: 之後（同一片雲有 65% 的點落在佔據圖的牆上 0.05 m 內，所以 TF 沒問題），
#: 它落在離最近牆 2.25 m 的空地上、離車正後方 0.45 m，**而且隨車一起移動**；
#: 佔據圖、建物 Mesh、18 個障礙 prim、13 個角色的位置都對不上。
#: 最可能是車體自身結構被自己的 RTX 光達打到（RTX 光達打的是算圖網格、
#: 不是物理碰撞體），不是環境障礙。
#: 寧可誠實標「不明」，也不要硬塞給最近的那一類 —— 那 4 幀被塞給了
#: 「走動行人」，而實際上最近的行人在 1.75 m 外。
UNEXPLAINED_SLACK_M = 0.5


def cylinder_surface_distance(p, centre, radius: float) -> float:
    """點到圓柱側面的水平距離。在裡面時回 0（不可為負）。

    ⚠ 回負值會讓 argmin 永遠選中它，把所有幀都歸給同一個障礙。
    """
    return max(0.0, math.hypot(p[0] - centre[0], p[1] - centre[1]) - radius)


def box_surface_distance(p, centre, half_x: float, half_y: float) -> float:
    """點到**軸對齊**方箱側面的水平距離。

    箱型障礙在 build_ros_graph 裡是用 map_to_world 擺位、不給旋轉，
    所以它的半寬是沿 **world** 軸的 —— 算距離要在 world frame 算。
    """
    dx = max(0.0, abs(p[0] - centre[0]) - half_x)
    dy = max(0.0, abs(p[1] - centre[1]) - half_y)
    return math.hypot(dx, dy)


def classify(dists):
    """``{類別: 距離}`` → 最近的那個類別名。距離是 None 的略過。

    static 沒有走動行人、dynamic 沒有箱型障礙，缺項是常態不是錯誤。
    """
    cand = [(d, n) for n, d in dists.items() if d is not None]
    return min(cand)[1] if cand else None


def align_offset(nav_xy, pose_samples, coarse: float = 0.5,
                 fine: float = 0.02):
    """求 nav CSV 的時間軸要加多少秒才對上模擬時間。

    nav CSV 的 ``t`` 是**該段導航開始後的 ROS 秒數**，pose.csv 的 ``t``
    是模擬時間；兩邊都記了車的位置，所以拿軌跡去對。

    回傳 ``(offset, rms)``。rms 是對齊後的位置殘差（m）——
    NDT 本身就有約 0.09 m 誤差，所以 rms 落在 0.1~0.3 m 是正常的；
    大於 1 m 表示沒對上，**不要相信**後面的分解。
    """
    import ros_graph_spec as S
    from pose_log import pose_at

    if not nav_xy or not pose_samples:
        return (0.0, float("inf"))
    probe = nav_xy[::max(1, len(nav_xy) // 200)]
    span = pose_samples[-1].t - pose_samples[0].t

    def cost(off: float) -> float:
        tot = 0.0
        for t, mx, my in probe:
            s = pose_at(pose_samples, t + off)
            gx, gy, _ = S.world_to_map(s.pos[0], s.pos[1], 0.0)
            tot += (gx - mx) ** 2 + (gy - my) ** 2
        return tot / len(probe)

    best, best_c = 0.0, float("inf")
    off = 0.0
    while off <= span:
        c = cost(off)
        if c < best_c:
            best, best_c = off, c
        off += coarse
    off = max(0.0, best - coarse)
    while off <= best + coarse:
        c = cost(off)
        if c < best_c:
            best, best_c = off, c
        off += fine
    return (best, math.sqrt(best_c))


def read_nav(run_dir: Path):
    """讀該趟的 nav CSV，回傳 ``{段名: [(t, map_x, map_y, min_range), ...]}``。"""
    out = {}
    for csv in sorted((run_dir / "nav").glob("*_leg*.csv")):
        rows = []
        for line in csv.read_text().splitlines()[1:]:
            f = line.split(",")
            if len(f) < 5:
                continue
            try:
                t, mx, my = float(f[0]), float(f[1]), float(f[2])
                mr = float(f[4])
            except ValueError:
                continue
            if math.isfinite(mr):
                rows.append((t, mx, my, mr))
        if rows:
            out[csv.stem] = rows
    return out


def character_home_positions(usd_path="assets/3floor_ver_1_ros_fixed.usda"):
    """讀 USD 裡 /World/Characters 每個角色的**原始** map 位置。

    static 情境的「走動人物」沒有被驅動，就停在這些位置上。
    """
    from pxr import Usd, UsdGeom

    import ros_graph_spec as S
    st = Usd.Stage.Open(str(usd_path))
    root = st.GetPrimAtPath("/World/Characters")
    if not (root and root.IsValid()):
        raise RuntimeError(f"{usd_path} 裡找不到 /World/Characters")
    cache = UsdGeom.XformCache()
    out = {}
    for ch in root.GetChildren():
        if ch.GetName() == "Biped_Setup":
            continue
        t = cache.GetLocalToWorldTransform(ch).ExtractTranslation()
        mx, my, _ = S.world_to_map(t[0], t[1], 0.0)
        out[ch.GetName()] = (mx, my, (t[0], t[1]))
    return out


def read_pgm(path):
    """讀 binary PGM（P5），回傳 ``(H, W)`` 的 uint8 陣列。

    自己讀是為了不替分析用的 venv 多裝一個影像庫；P5 很單純，
    但**註解行可以出現在任何一個空白之後**（這份圖就是 GIMP 存的，
    magic 之後第一行就是註解），所以要逐 token 掃、遇到 # 跳到行尾。
    """
    import numpy as np

    raw = Path(path).read_bytes()
    if raw[:2] != b"P5":
        raise ValueError(f"{path} 不是 binary PGM（開頭 {raw[:2]!r}）")
    i, vals = 2, []
    while len(vals) < 3:
        if i >= len(raw):
            raise ValueError(f"{path} 的 PGM 標頭不完整")
        c = raw[i:i + 1]
        if c == b"#":
            i = raw.index(b"\n", i) + 1
        elif c.isspace():
            i += 1
        else:
            j = i
            while not raw[j:j + 1].isspace():
                j += 1
            vals.append(int(raw[i:j]))
            i = j
    i += 1                                     # 跳過 maxval 後的單一空白
    w, h, _maxv = vals
    return np.frombuffer(raw, dtype=np.uint8, count=w * h,
                         offset=i).reshape(h, w)


def _wall_distance_field(pgm="map/4v3F.pgm", yml="map/4v3F.yaml"):
    """回傳吃 map 座標、回傳「到最近佔據格距離」的查詢函式。"""
    import numpy as np
    import yaml
    from scipy import ndimage

    cfg = yaml.safe_load(Path(yml).read_text())
    res = float(cfg["resolution"])
    ox, oy = float(cfg["origin"][0]), float(cfg["origin"][1])
    img = read_pgm(pgm)
    H, W = img.shape
    occ = ((255 - img.astype(np.float32)) / 255.0) > float(cfg["occupied_thresh"])
    dist = ndimage.distance_transform_edt(~occ) * res

    def query(mx, my):
        px = int(round((mx - ox) / res))
        py = H - 1 - int(round((my - oy) / res))
        if not (0 <= px < W and 0 <= py < H):
            return None
        return float(dist[py, px])

    return query


def decompose_run(run_dir: Path, wall_q, homes):
    """回傳這一趟的分解結果，或 None（資料不足）。"""
    import ros_graph_spec as S
    from character_colliders import too_close_to_robot
    from pose_log import parse_crowd_rows, parse_rows, pose_at
    from scene_variants import variant
    from scenarios import scenario_config

    meta = json.loads((run_dir / "run.json").read_text())
    idx = int(meta.get("run_index", 0))
    scen = scenario_config(meta.get("scenario", "mixed"))
    nav = read_nav(run_dir)
    if not nav or not (run_dir / "pose.csv").exists():
        return None
    poses = parse_rows((run_dir / "pose.csv").read_text().splitlines())
    if len(poses) < 2:
        return None
    rob0 = (poses[0].pos[0], poses[0].pos[1])

    var = variant(idx) if idx >= 1 else None
    boxes, standing, static_peds = [], [], []
    if var is not None:
        if scen.obstacles_enabled:
            for o in var.obstacles:
                if o.kind == "box":
                    wx, wy, _ = S.map_to_world(o.map_x, o.map_y, 0.0)
                    boxes.append((wx, wy, o.size_x / 2.0, o.size_y / 2.0))
        # 站立人物在**三個情境都在場**（place_standing 不看 obstacles_enabled）
        standing = [(p.map_x, p.map_y) for p in var.standing]
        if not scen.walks_enabled:
            # 走動人物沒被驅動 → 停在 USD 原位（太靠近車的那些已被停用）
            for w in var.walks:
                h = homes.get(w.name)
                if h and not too_close_to_robot(h[2], rob0):
                    static_peds.append((h[0], h[1]))

    crowd_by_frame = defaultdict(list)
    if (run_dir / "crowd.csv").exists():
        for c in parse_crowd_rows((run_dir / "crowd.csv").read_text().splitlines()):
            crowd_by_frame[round(c.t * 30.0)].append((c.x, c.y))

    closest = defaultdict(int)
    near = defaultdict(int)
    resid, n, worst = [], 0, (float("inf"), None)
    legs = {}
    for leg, rows in sorted(nav.items()):
        off, rms = align_offset([(t, mx, my) for t, mx, my, _ in rows], poses)
        legs[leg] = (off, rms)
        for t, mx, my, measured in rows:
            s = pose_at(poses, t + off)
            gx, gy, _ = S.world_to_map(s.pos[0], s.pos[1], 0.0)
            d = {}
            peds = crowd_by_frame.get(round((t + off) * 30.0))
            d["走動行人"] = (min(cylinder_surface_distance((gx, gy), p,
                                                        PED_RADIUS_M)
                                for p in peds) if peds else None)
            fixed = standing + static_peds
            d["站立行人"] = (min(cylinder_surface_distance((gx, gy), p,
                                                        STANDING_RADIUS_M)
                                for p in fixed) if fixed else None)
            d["箱型障礙"] = (min(box_surface_distance(
                (s.pos[0], s.pos[1]), (bx, by), hx, hy)
                for bx, by, hx, hy in boxes) if boxes else None)
            d["牆"] = wall_q(gx, gy)
            who = classify(d)
            if who is None:
                continue
            pred = min(v for v in d.values() if v is not None)
            n += 1
            closest[who] += 1
            resid.append(pred - measured)
            if measured <= NEAR_THRESHOLD_M:
                near["來源不明" if pred - measured > UNEXPLAINED_SLACK_M
                     else who] += 1
            if measured < worst[0]:
                worst = (measured, who, {k: v for k, v in d.items()
                                         if v is not None})
    if not n:
        return None
    resid.sort()
    return {"tag": meta["tag"], "scenario": meta.get("scenario"),
            "run_index": idx, "samples": n,
            "closest": dict(closest), "near": dict(near),
            "near_total": sum(near.values()),
            "resid_med": resid[len(resid) // 2],
            "resid_p05": resid[int(len(resid) * 0.05)],
            "resid_p95": resid[int(len(resid) * 0.95)],
            "legs": legs, "worst": worst}


def main(root: Path) -> int:
    wall_q = _wall_distance_field()
    homes = character_home_positions()
    rows = []
    hdr = f"{'tag':28s}{'取樣':>5s}" + "".join(f"{c:>9s}" for c in CATEGORIES)
    print(hdr + f"{'對齊殘差':>10s}{'≤0.45m 的來源':>16s}")
    print("-" * (len(hdr) + 30))
    for d in run_dirs(root):
        r = decompose_run(d, wall_q, homes)
        if r is None:
            continue
        rows.append(r)
        n = max(1, r["samples"])
        cells = "".join(f"{100 * r['closest'].get(c, 0) / n:8.0f}%"
                        for c in CATEGORIES)
        ns = ("—" if not r["near_total"] else
              " ".join(f"{k}:{v}" for k, v in sorted(r["near"].items())))
        print(f"{r['tag']:28s}{r['samples']:5d}{cells}"
              f"{r['resid_med']:+9.2f}m{ns:>16s}")
    if not rows:
        print("（沒有可分析的趟）")
        return 1

    tot = sum(r["samples"] for r in rows)
    agg, near_agg = defaultdict(int), defaultdict(int)
    for r in rows:
        for k, v in r["closest"].items():
            agg[k] += v
        for k, v in r["near"].items():
            near_agg[k] += v
    print(f"\n合計 {len(rows)} 趟 / {tot} 個取樣點（10 Hz）")
    print("  「最近的東西」是誰：" + "　".join(
        f"{k} {100 * v / tot:.1f}%" for k, v in
        sorted(agg.items(), key=lambda kv: -kv[1])))
    nt = sum(near_agg.values())
    if nt:
        print(f"  實際量到 ≤{NEAR_THRESHOLD_M} m 的取樣點共 {nt} 個，最近的是：")
        for k, v in sorted(near_agg.items(), key=lambda kv: -kv[1]):
            print(f"      {k:8s} {v:4d} 個（{100 * v / nt:.0f}%）")
    else:
        print(f"  沒有任何取樣點量到 ≤{NEAR_THRESHOLD_M} m")

    bad = [r for r in rows if abs(r["resid_med"]) > 0.5]
    print(f"\n  預測−實測 殘差：中位 {min(r['resid_med'] for r in rows):+.2f} ~ "
          f"{max(r['resid_med'] for r in rows):+.2f} m"
          + (f"　⚠ {len(bad)} 趟中位殘差超過 0.5 m，那幾趟不可信" if bad else
             "　（逐趟中位都在 ±0.5 m 內，分解可信）"))
    print(f"    殘差為正 = 預測比實測遠（場上有沒登記的東西）；"
          f"為負 = 預測比實測近（光達沒打到那個東西）")
    print(f"    p95 最大 {max(r['resid_p95'] for r in rows):+.2f} m　"
          f"p05 最小 {min(r['resid_p05'] for r in rows):+.2f} m")

    print("\n  每趟最接近的那一刻：")
    for r in sorted(rows, key=lambda r: r["worst"][0])[:12]:
        m, who, ds = r["worst"]
        detail = "　".join(f"{k} {v:.2f}" for k, v in ds.items())
        print(f"    {r['tag']:28s} 實測 {m:.2f} m → {who}　({detail})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(Path(sys.argv[1] if len(sys.argv) > 1
                               else "recordings")))
