"""在 Isaac 裡建三視角相機並錄成 PNG 序列（執行期）。

為什麼不用 GUI 螢幕錄影：見 sim_cameras 模組說明。

⚠ 保真度風險：policy 的推論迴圈跑在**牆鐘**上。算圖若拖慢模擬（RTF < 1），
每個模擬秒會拿到比實車更多次決策，錄下來的行為就不代表真實表現。
所以 `record_every` 預設每 2 個 render tick 才抓一幀（30 fps → 15 fps），
並且 run_isaac_sim 會印出 RTF 供事後判讀。

⚠ 這支**不補光**。2026-09-22 曾因為畫面全黑而加了一盞跟車球燈，那是誤診：
   真正的原因是 RTX 的 `/rtx/rendermode` 沒有被渲染器吃進去，所有表面都
   算不出光照（只剩背景天空）。證據：把 3e6 的球燈擺在車正上方 1.2 m，
   畫面**位元不變**；而把 `/rtx/rendermode` 重新設一次 "RaytracedLighting"，
   同一個場景的俯視圖立刻從平均 0.00 跳到 53.5。
   修正在 run_isaac_sim（見該處註解），這裡不該再有補光邏輯 ——
   場景本身的 DomeLight/DistantLight 就夠亮，而且補光球還會擋住俯視鏡頭。
"""

from __future__ import annotations

import math
from pathlib import Path

from sim_cameras import (CAMERAS, camera_pose, look_at_rotation, pull_fraction,
                         pulled_eye, side_rays, smooth_pull, wall_ray, _apply)

CAMERA_ROOT = "/World/RecCams"

#: 避牆射線要忽略的東西：只該被**建築**擋，不是被人、障礙、車自己、
#: 地圖補丁（隱形天花板層）擋 —— 那些不會讓畫面變黑白，拉近反而會抖。
RAY_IGNORE_PREFIXES = ("/World/Characters", "/World/SimObstacles",
                       "/World/charger_rover4_5_0", "/World/MapPatch",
                       CAMERA_ROOT)


GRID_IGNORE_PREFIXES = ("/World/Characters", "/World/charger_rover4_5_0",
                        "/World/MapPatch", CAMERA_ROOT)


def build_wall_grid(stage, floor_z: float, center_xy, radius: float = 60.0):
    """從**算圖網格**建 2D 牆面格網（見 wall_grid 模組說明）。

    ⚠ 2026-09-24：只靠 PhysX 射線時，門/內凹處沒有碰撞體，相機照樣進牆。
    只取車附近 ``radius`` 內、跨過相機高度帶的三角形。
    """
    import numpy as np
    from pxr import Usd, UsdGeom
    from wall_grid import SLICE_HEIGHTS_M, WallGrid, slice_segments, triangles_from_mesh

    zlo = floor_z + min(SLICE_HEIGHTS_M)
    zhi = floor_z + max(SLICE_HEIGHTS_M)
    cache = UsdGeom.XformCache()
    segs = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        # 道具（置物櫃等）要算：相機埋進櫃子裡一樣全黑。人會動、車是自己，不算。
        if str(prim.GetPath()).startswith(GRID_IGNORE_PREFIXES):
            continue
        if prim.GetTypeName() != "Mesh":
            continue
        img = UsdGeom.Imageable(prim)
        if img.ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue                                   # 隱形碰撞體（障礙、補丁）不是牆
        m = UsdGeom.Mesh(prim)
        pts = m.GetPointsAttr().Get()
        cnt = m.GetFaceVertexCountsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        if not pts or cnt is None or idx is None:
            continue
        P = np.asarray(pts, dtype=float)
        X = np.asarray(cache.GetLocalToWorldTransform(prim), dtype=float)
        P = P @ X[:3, :3] + X[3, :3]                   # pxr row-vector
        if (np.hypot(P[:, 0] - center_xy[0], P[:, 1] - center_xy[1]).min() > radius
                or P[:, 2].max() < zlo or P[:, 2].min() > zhi):
            continue
        tris = triangles_from_mesh(P, cnt, idx)
        tz = tris[:, :, 2]
        tris = tris[(tz.max(1) >= zlo) & (tz.min(1) <= zhi)]
        for h in SLICE_HEIGHTS_M:
            segs.append(slice_segments(tris, floor_z + h))
    segs = np.concatenate(segs) if segs else np.zeros((0, 2, 2))
    return WallGrid.from_segments(segs), len(segs)


def _nearest_wall_hit(origin, unit_dir, length):
    """PhysX 射線：回傳最近的建築擋點距離，沒有回 None。"""
    from omni.physx import get_physx_scene_query_interface

    best = [None]

    def report(hit):
        path = str(hit.collision)
        if not path.startswith(RAY_IGNORE_PREFIXES):
            if best[0] is None or hit.distance < best[0]:
                best[0] = hit.distance
        return True                                   # 繼續收集

    get_physx_scene_query_interface().raycast_all(
        origin, unit_dir, length, report)
    return best[0]


class CameraRecorder:
    """三視角同時錄影。每幀更新相機位姿並寫出 PNG。"""

    def __init__(self, stage, out_dir: Path, width: int = 1280, height: int = 720,
                 record_every: int = 1):
        from pxr import UsdGeom

        self._stage = stage
        self._out = Path(out_dir)
        self._every = max(1, record_every)
        self._n = 0
        self._written = 0
        self._index: list[tuple[int, float]] = []
        self._cams = []
        self._writers = []
        self._pull: dict[str, float] = {}             # 各相機目前的拉近比例
        self.pulled_frames: dict[str, int] = {}       # 統計：被牆擋而拉近的幀數
        self._ray_warned = False
        self._grid = None                             # 第一次 update_poses 時建
        self._grid_failed = False

        UsdGeom.Scope.Define(stage, CAMERA_ROOT)

        import omni.replicator.core as rep

        # ⚠ 一次接好，不要「先接暖機 writer 再 detach 換正式 writer」——
        #   2026-09-22 試過那個寫法，畫面反而從「只剩天空」變成整張全 0。
        for spec in CAMERAS:
            path = f"{CAMERA_ROOT}/{spec.name}"
            cam = UsdGeom.Camera.Define(stage, path)
            cam.CreateFocalLengthAttr(float(spec.focal_length_mm))
            cam.CreateClippingRangeAttr((0.05, 500.0))
            cam.MakeMatrixXform()
            sub = self._out / spec.name
            sub.mkdir(parents=True, exist_ok=True)
            rp = rep.create.render_product(path, (width, height),
                                           name=f"rp_{spec.name}")
            w = rep.WriterRegistry.get("BasicWriter")
            w.initialize(output_dir=str(sub), rgb=True)
            w.attach([rp])
            self._cams.append((spec, cam))
            self._writers.append(w)

    def __len__(self) -> int:
        return len(self._cams)

    @property
    def frames_written(self) -> int:
        return self._written

    def update_poses(self, robot_xy, yaw: float, floor_z: float) -> None:
        """把三台相機挪到相對車體的正確位置。"""
        from pxr import Gf, UsdGeom

        if self._grid is None and not self._grid_failed:
            import time as _t
            t0 = _t.perf_counter()
            try:
                self._grid, nseg = build_wall_grid(self._stage, floor_z, robot_xy)
                print(f"[camera] 牆面格網：{nseg} 段、{int(self._grid.occ.sum())} 格佔據"
                      f"（{_t.perf_counter() - t0:.1f} s）", flush=True)
            except Exception as e:
                self._grid_failed = True
                print(f"[camera] ⚠ 牆面格網建立失敗，只用 PhysX 射線：{e!r}", flush=True)

        for spec, cam in self._cams:
            eye, (axis, ang) = camera_pose(spec, robot_xy, yaw, floor_z)
            if spec.avoid_walls:
                origin, udir, length = wall_ray(spec, robot_xy, yaw, floor_z)
                try:
                    hit = _nearest_wall_hit(origin, udir, length)
                except Exception as e:            # 射線壞了不中斷錄影，但要講
                    hit = None
                    if not self._ray_warned:
                        print(f"[camera] ⚠ 避牆射線失敗，{spec.name} 不避牆：{e!r}",
                              flush=True)
                        self._ray_warned = True
                if self._grid is not None:
                    # 射線是水平的（同高），格網距離＝射線距離
                    gh = self._grid.first_hit(origin[0], origin[1], udir[0], udir[1],
                                              length, skip=0.4)
                    if gh is not None and (hit is None or gh < hit):
                        hit = gh
                target = pull_fraction(length, hit)
                # 兩側射線：牆角在旁邊時也要拉近（比例套回中線）
                for so, sd, sl in side_rays(spec, robot_xy, yaw, floor_z):
                    sh = None
                    try:
                        sh = _nearest_wall_hit(so, sd, sl)
                    except Exception:
                        pass
                    if self._grid is not None:
                        g2 = self._grid.first_hit(so[0], so[1], sd[0], sd[1], sl, skip=0.4)
                        if g2 is not None and (sh is None or g2 < sh):
                            sh = g2
                    target = min(target, pull_fraction(sl, sh))
                f = smooth_pull(self._pull.get(spec.name, 1.0), target)
                self._pull[spec.name] = f
                if f < 0.999:
                    self.pulled_frames[spec.name] = self.pulled_frames.get(spec.name, 0) + 1
                    eye = pulled_eye(origin, udir, length, f)
                    tx, ty, tz = _apply(spec.target_offset, yaw, spec.target_follow_yaw)
                    axis, ang = look_at_rotation(
                        eye, (robot_xy[0] + tx, robot_xy[1] + ty, floor_z + tz))
            m = Gf.Matrix4d(1.0).SetRotate(
                Gf.Rotation(Gf.Vec3d(*axis), ang))
            m = m * Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(*eye))
            ops = [o for o in UsdGeom.Xformable(cam).GetOrderedXformOps()
                   if o.GetOpType() == UsdGeom.XformOp.TypeTransform]
            if ops:
                ops[0].Set(m)

    def step(self, sim_time: float = 0.0) -> bool:
        """每個**算圖幀**呼叫一次，記下「幀序號 ↔ 模擬時間」。

        ⚠ 不要呼叫 rep.orchestrator.step()：BasicWriter 掛上 render product
        後**每個算圖幀會自動寫檔**（實測 15 s 產出 447 幀 = 30 fps）。
        再手動觸發會重複算圖且序號對不上。這支只負責記帳。

        rosbag 用 use_sim_time 錄，兩者靠模擬時間就能精確對齊，
        事後剪輯不必猜。
        """
        self._index.append((self._written, sim_time))
        self._written += 1
        return True

    def close(self) -> None:
        # 幀↔模擬時間對照表，與 rosbag 同步用
        try:
            with open(self._out / "frame_times.csv", "w") as f:
                f.write("frame,sim_time\n")
                for i, t in self._index:
                    f.write(f"{i},{t:.4f}\n")
        except Exception:
            pass
        for w in self._writers:
            try:
                w.detach()
            except Exception:
                pass

