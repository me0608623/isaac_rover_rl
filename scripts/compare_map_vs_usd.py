#!/usr/bin/env python3
"""離線比對 NDT 地圖（真實世界掃描）與 USD 模型的幾何差異。

為什麼要比：NDT 的地圖是**真實世界掃出來的**，而模擬的點雲是從 **USD 模型**
raycast 出來的。兩者若有幾何落差，NDT 就是在把「模型的樣子」對齊到
「真實世界的地圖」—— 系統性對不上，且無論怎麼調 NDT 參數都修不好。

這支不需要 Isaac，也不依賴 NDT 的解，所以量到的是**純幾何**差異。

線索：車靜止、無角色時 NDT 收斂後傾角穩在 1.62° 而非 0°，這個固定偏差
需要解釋（2026-09-21 實測）。
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_graph_spec as S


def read_pcd(path: Path) -> np.ndarray:
    """讀 binary PCD 的 xyz（忽略 intensity）。"""
    with open(path, "rb") as f:
        fields, sizes, counts, npts, data_fmt = None, None, None, 0, ""
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("SIZE"):
                sizes = [int(v) for v in line.split()[1:]]
            elif line.startswith("COUNT"):
                counts = [int(v) for v in line.split()[1:]]
            elif line.startswith("POINTS"):
                npts = int(line.split()[1])
            elif line.startswith("DATA"):
                data_fmt = line.split()[1]
                break
        if data_fmt != "binary":
            raise RuntimeError(f"只支援 binary PCD，收到 {data_fmt}")
        stride = sum(s * c for s, c in zip(sizes, counts))
        buf = f.read(npts * stride)
    arr = np.frombuffer(buf, dtype=np.uint8).reshape(npts, stride)
    out = np.empty((npts, 3), dtype=np.float32)
    off = 0
    for name, s_, c_ in zip(fields, sizes, counts):
        if name in ("x", "y", "z"):
            out[:, "xyz".index(name)] = arr[:, off:off + 4].copy().view(np.float32).ravel()
        off += s_ * c_
    return out.astype(np.float64)


def sample_usd_surfaces(usd_path: str, root: str, n_target: int,
                        exclude=("Characters", "charger_rover")) -> np.ndarray:
    """在 USD 的三角面上依面積取樣（world frame）。

    只取頂點會嚴重偏向細碎結構 —— 一面大牆可能只有 4 個頂點，
    一根裝飾柱卻有上百個。依面積取樣才反映光達實際會打到的表面。
    """
    import os

    from pxr import Gf, Usd, UsdGeom
    err = os.dup(2)
    os.close(2)
    os.open(os.devnull, os.O_WRONLY)
    stage = Usd.Stage.Open(usd_path)
    os.dup2(err, 2)

    tris_v: list[np.ndarray] = []
    cache = UsdGeom.XformCache()
    for prim in stage.Traverse(Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if prim.GetTypeName() != "Mesh":
            continue
        p = str(prim.GetPath())
        if root not in p or any(e in p for e in exclude):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idx:
            continue
        m = cache.GetLocalToWorldTransform(prim)
        w = np.array([list(m.Transform(Gf.Vec3f(v))) for v in pts], dtype=np.float64)
        cur = 0
        for n in counts:
            face = [idx[cur + k] for k in range(n)]
            cur += n
            for k in range(1, n - 1):       # 扇形三角化
                tris_v.append(w[[face[0], face[k], face[k + 1]]])
    if not tris_v:
        return np.zeros((0, 3))
    T = np.stack(tris_v)                     # (N, 3, 3)
    e1, e2 = T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]
    area = 0.5 * np.linalg.norm(np.cross(e1, e2), axis=1)
    if area.sum() <= 0:
        return np.zeros((0, 3))
    prob = area / area.sum()
    pick = np.random.default_rng(0).choice(len(T), size=n_target, p=prob)
    u = np.random.default_rng(1).random((n_target, 1))
    v = np.random.default_rng(2).random((n_target, 1))
    over = (u + v) > 1.0
    u[over], v[over] = 1.0 - u[over], 1.0 - v[over]
    return T[pick, 0] + u * e1[pick] + v * e2[pick]


def world_to_map(xy: np.ndarray) -> np.ndarray:
    yaw = S.WORLD_TO_MAP_YAW_RAD
    tx, ty = S.WORLD_TO_MAP_TRANSLATION
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([xy[:, 0] * c - xy[:, 1] * s + tx,
                     xy[:, 0] * s + xy[:, 1] * c + ty], axis=1)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pcd", default="src/ndt_localizer/map/3F_314.pcd")
    ap.add_argument("--usd", default="/home/aa/Ros/charge_rl/assets/3F/3floor_ver_1.usd")
    ap.add_argument("--root", default="/World/Env_0")
    ap.add_argument("--samples", type=int, default=400000)
    ap.add_argument("--cx", type=float, default=-7.0, help="比對中心 map x")
    ap.add_argument("--cy", type=float, default=5.0, help="比對中心 map y")
    ap.add_argument("--half", type=float, default=10.0, help="比對範圍半寬 m")
    ap.add_argument("--slice-lo", type=float, default=0.5, help="取樣高度下限（map z）")
    ap.add_argument("--slice-hi", type=float, default=1.5)
    args = ap.parse_args()

    M = read_pcd(Path(args.pcd))
    print(f"地圖 {len(M)} 點   x[{M[:,0].min():.1f},{M[:,0].max():.1f}] "
          f"y[{M[:,1].min():.1f},{M[:,1].max():.1f}] z[{M[:,2].min():.1f},{M[:,2].max():.1f}]")

    U = sample_usd_surfaces(args.usd, args.root, args.samples)
    if len(U) == 0:
        print("[FAIL] USD 取不到面")
        return 1
    Uxy = world_to_map(U[:, :2])
    # USD world z → map z（只有平移，見 WORLD_TO_MAP_Z_OFFSET）
    Uz = U[:, 2] + S.WORLD_TO_MAP_Z_OFFSET
    print(f"USD  {len(U)} 取樣點（面積加權）→ map "
          f"x[{Uxy[:,0].min():.1f},{Uxy[:,0].max():.1f}] "
          f"y[{Uxy[:,1].min():.1f},{Uxy[:,1].max():.1f}] z[{Uz.min():.1f},{Uz.max():.1f}]")

    def crop(xy, z):
        m = ((np.abs(xy[:, 0] - args.cx) < args.half)
             & (np.abs(xy[:, 1] - args.cy) < args.half)
             & (z >= args.slice_lo) & (z < args.slice_hi))
        return xy[m]

    mm = crop(M[:, :2], M[:, 2])
    uu = crop(Uxy, Uz)
    print(f"\n比對區域 map({args.cx:+.1f},{args.cy:+.1f}) ±{args.half} m，"
          f"高度 [{args.slice_lo},{args.slice_hi}) m")
    print(f"  地圖 {len(mm)} 點   USD {len(uu)} 點")
    if len(mm) < 500 or len(uu) < 500:
        print("[WARN] 區域內點太少，換個中心或放寬高度")
        return 0

    # 2D 占據網格互相關 → 找最佳平移（有系統性偏移就會顯現）
    V = 0.10
    def grid(xy):
        i = np.floor((xy[:, 0] - (args.cx - args.half)) / V).astype(int)
        j = np.floor((xy[:, 1] - (args.cy - args.half)) / V).astype(int)
        n = int(2 * args.half / V)
        ok = (i >= 0) & (i < n) & (j >= 0) & (j < n)
        g = np.zeros((n, n), dtype=np.float64)
        np.add.at(g, (i[ok], j[ok]), 1.0)
        return (g > 0).astype(np.float64)

    A, B = grid(mm), grid(uu)
    best, bij = -1.0, (0, 0)
    R = int(1.0 / V)                     # 搜尋 ±1.0 m
    for di in range(-R, R + 1):
        for dj in range(-R, R + 1):
            sh = np.roll(np.roll(B, di, axis=0), dj, axis=1)
            s = float((A * sh).sum())
            if s > best:
                best, bij = s, (di, dj)
    base = float((A * B).sum())
    print(f"  占據格重疊：原位 {base:.0f}    最佳平移後 {best:.0f}")
    print(f"  最佳平移 = ({bij[0]*V:+.2f}, {bij[1]*V:+.2f}) m")
    if math.hypot(bij[0] * V, bij[1] * V) > 0.15:
        print("  ⚠ 地圖與 USD 有系統性平移偏差 —— NDT 會一直在補這個差")
    else:
        print("  → 水平方向沒有明顯系統性偏移")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
