#!/usr/bin/env python3
"""啟動 Isaac Sim 載入修正後的走廊場景，並跑 ROS 2 bridge。

這支只負責「把模擬跑起來」；ROS 那一側（NDT / routing / policy / 降級節點）
由 sim_ws/launch 另外啟動，兩邊靠 topic 溝通。

用法：
    source sim_ws/setup_sim_env.sh
    python3 sim_ws/scripts/run_isaac_sim.py                 # headless
    python3 sim_ws/scripts/run_isaac_sim.py --gui           # 開視窗
    python3 sim_ws/scripts/run_isaac_sim.py --seconds 30    # 跑完自動結束（驗證用）
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

DEFAULT_USD = Path(__file__).resolve().parents[1] / "assets" / "3floor_ver_1_ros_fixed.usda"

#: VLP-16 @ rpm 600。headless 時渲染頻率只需跟上它。
#: ⚠ 2026-09-21 實測：render_every=0（完全不渲染）會讓整張 Action Graph 停擺 ——
#:   OnPlaybackTick 綁在 app update / render tick 上，不是物理步。實測現象是
#:   /velodyne_points_ideal 從 topic list 消失，/clock 與 /odom_gt 一則訊息都不發。
S_LIDAR_HZ = 10.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", type=Path, default=DEFAULT_USD)
    ap.add_argument("--gui", action="store_true", help="開視窗（預設 headless）")
    ap.add_argument("--seconds", type=float, default=0.0, help=">0 時跑滿即結束")
    ap.add_argument("--free-run", action="store_true",
                    help="不做實時節流，讓模擬盡可能快。預設會節流到 1.0x —— \
"
                         "policy 的推論迴圈是照牆鐘跑的，模擬若比實時快，每個模擬秒\
"
                         "拿到的決策次數就少於實車，控制迴圈時序會對不上。")
    ap.add_argument("--physics-hz", type=float, default=60.0)
    ap.add_argument("--render-hz", type=float, default=30.0)
    ap.add_argument("--render-every", type=int, default=-1,
                    help="每 N 個 physics step 才渲染一次；0=完全不渲染；"
                         "-1(預設)=headless 時自動用 0、GUI 時用 1。"
                         "PhysX Lidar 走物理 raycast 不需渲染，headless 下渲染是純浪費，"
                         "而且正好會去跟其他 GPU 工作搶資源。")
    ap.add_argument("--no-character-colliders", dest="character_colliders",
                    action="store_false",
                    help="不替 People 角色套碰撞體（角色就只是看得到、打不到）")
    ap.add_argument("--no-pedestrians", action="store_true",
                    help="不驅動移動行人（做定位基準或需要完全靜態場景時用）")
    args = ap.parse_args()

    if not args.usd.exists():
        print(f"[run_isaac_sim] 找不到 USD: {args.usd}", file=sys.stderr)
        return 2

    from isaacsim import SimulationApp

    # headless 下沒有任何東西需要那些像素（PhysX Lidar 走 raycast，也沒有 camera
    # 在發布），但 SimulationApp 預設仍以 1280x720 在算。壓到最小可省下大量 GPU。
    # ⚠ 渲染不能完全關掉 —— OnPlaybackTick 綁在 render tick 上（見 S_LIDAR_HZ 註解）。
    render_res = (128, 128) if not args.gui else (1280, 720)
    app = SimulationApp({
        "headless": not args.gui,
        "renderer": "RayTracedLighting",
        "width": render_res[0],
        "height": render_res[1],
    })

    # ROS 2 bridge 必須在開 stage 前啟用，Action Graph 裡的 ROS 節點才註冊得到型別
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")
    app.update()

    import omni.usd
    from isaacsim.core.utils.stage import is_stage_loading

    print(f"[run_isaac_sim] 開啟 {args.usd}")
    omni.usd.get_context().open_stage(str(args.usd))
    while is_stage_loading():
        app.update()
    print("[run_isaac_sim] stage 載入完成")

    from isaacsim.core.api import SimulationContext

    sim = SimulationContext(physics_dt=1.0 / args.physics_hz,
                            rendering_dt=1.0 / args.render_hz,
                            stage_units_in_meters=1.0)
    sim.initialize_physics()
    sim.play()

    def _dump_poses(tag: str) -> None:
        """印出執行期的實際世界位姿。

        靜態讀 USD 會被物理關節與求解器覆蓋（velodyne 帶 PhysicsRigidBodyAPI），
        所以感測器高度、車體是否貼地這類問題只能在 play 之後量。
        """
        from pxr import UsdGeom
        import omni.usd as _ou
        st = _ou.get_context().get_stage()
        cache = UsdGeom.XformCache()
        R = "/World/charger_rover4_5_0/charger_rover_urdf5"
        out = []
        for name in ("base_footprint", "base_link", "velodyne", "left_wheel"):
            pr = st.GetPrimAtPath(f"{R}/{name}")
            if not pr or not pr.IsValid():
                continue
            t = cache.GetLocalToWorldTransform(pr).ExtractTranslation()
            out.append(f"{name}=({t[0]:+.4f},{t[1]:+.4f},{t[2]:+.4f})")
        print(f"[run_isaac_sim] {tag}: " + "  ".join(out), flush=True)

    _dump_poses("play 之前")

    # People 角色：執行期套三角網格碰撞體，PhysX 光達才打得到人體輪廓。
    # 必須在這裡做 —— 角色網格在 CDN 參照底下，離線 usd-core 看不到。
    if args.character_colliders:
        import sys as _s0
        _s0.path.insert(0, str(Path(__file__).resolve().parent))
        from character_colliders import apply_mesh_colliders
        from pxr import UsdGeom as _UG
        _st = omni.usd.get_context().get_stage()
        _rb = _st.GetPrimAtPath(
            "/World/charger_rover4_5_0/charger_rover_urdf5/base_footprint")
        _rxy = None
        if _rb and _rb.IsValid():
            _t = _UG.XformCache().GetLocalToWorldTransform(_rb).ExtractTranslation()
            _rxy = (_t[0], _t[1])
        n_c, n_m, skipped = apply_mesh_colliders(_st, "/World/Characters", _rxy)
        print(f"[run_isaac_sim] 角色三角網格碰撞體：{n_c} 人 / {n_m} 個 mesh"
              + (f"　跳過(太靠近車) {skipped}" if skipped else ""))

    # 移動行人：kinematic rigid body，位姿每個物理步由我們覆寫。
    driver = None
    if not args.no_pedestrians:
        import sys as _sys
        _sys.path.insert(0, str(Path(__file__).resolve().parent))
        import ros_graph_spec as _S
        from build_ros_graph import MOVING_ROOT, measure_corridor_floor_top
        from obstacle_driver import MovingObstacleDriver

        stage = omni.usd.get_context().get_stage()
        driver = MovingObstacleDriver(
            stage, _S.DEFAULT_MOVING_OBSTACLES,
            measure_corridor_floor_top(stage), MOVING_ROOT,
        )
        print(f"[run_isaac_sim] 移動行人 {len(driver)} 名"
              + ("" if len(driver) else "  ⚠ USD 裡沒有行人 prim，請重跑 build_ros_graph.py"))

    print(f"[run_isaac_sim] 開始模擬  physics={args.physics_hz} Hz  render={args.render_hz} Hz")

    render_every = args.render_every
    if render_every < 0:
        # 實測（128x128 headless、RTX 5090 且有其他 GPU 工作競爭）：
        #   render_every=6 → 點雲 2.9 Hz, RTF 0.99
        #   render_every=2 → 點雲 6.7 Hz, RTF 0.98
        #   render_every=1 → 點雲 10.1 Hz（VLP-16 正確值）, RTF 0.89  ← 採用
        # PhysX Lidar 的掃描累積綁在 render tick 上，所以要拿到 10 Hz 就得每步都渲染。
        render_every = 1
    print(f"[run_isaac_sim] render_every={render_every} "
          f"({'不渲染' if render_every == 0 else f'每 {render_every} 步渲染一次'})")

    steps = 0
    t_start = time.perf_counter()
    t_report = t_start
    sim_report = sim.current_time
    try:
        while app.is_running():
            if driver is not None:
                driver.update(sim.current_time)
            do_render = render_every > 0 and steps % render_every == 0
            sim.step(render=do_render)
            steps += 1

            # 實時節流：模擬跑太快時等一下，讓 sim time 貼齊牆鐘。
            if not args.free_run:
                lead = sim.current_time - (time.perf_counter() - t_start)
                if lead > 0.002:
                    time.sleep(lead)

            if steps in (1, 30, 120, 600):
                _dump_poses(f"step {steps}")
            if steps % int(args.physics_hz) == 0:
                now = time.perf_counter()
                # ⚠ 即時比 = 模擬時間前進量 / 牆鐘前進量。不能用 physics_hz/牆鐘 ——
                #   rendering_dt(1/30) 比 physics_dt(1/60) 大時，sim.step() 實際
                #   前進的是 rendering_dt，用步數換算會高估 60 倍（2026-09-21 誤導過）。
                rtf = ((sim.current_time - sim_report) / (now - t_report)
                       if now > t_report else 0.0)
                t_report, sim_report = now, sim.current_time
                print(f"[run_isaac_sim] sim_time={sim.current_time:8.2f} s  "
                      f"steps={steps}  RTF={rtf:.2f}x", flush=True)
            if args.seconds > 0 and sim.current_time >= args.seconds:
                print(f"[run_isaac_sim] 已達 {args.seconds} s，結束")
                break
    except KeyboardInterrupt:
        print("[run_isaac_sim] 中斷")
    finally:
        sim.stop()
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
