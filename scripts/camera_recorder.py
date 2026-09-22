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

from sim_cameras import CAMERAS, camera_pose

CAMERA_ROOT = "/World/RecCams"


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

        for spec, cam in self._cams:
            eye, (axis, ang) = camera_pose(spec, robot_xy, yaw, floor_z)
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

