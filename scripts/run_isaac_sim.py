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
    ap.add_argument("--character-mode", choices=("parts", "whole", "off"),
                    default="parts",
                    help="People 角色的光達可見性："
                         "parts=逐部位碰撞體（跟著骨架動畫擺動，預設）；"
                         "whole=整具一塊（凍結在 T-pose）；off=不套（看得到打不到）")
    ap.add_argument("--part-approx", choices=("convexHull", "none"),
                    default="convexHull",
                    help="部位碰撞體的近似。convexHull(預設)便宜且逐部位後幾乎不損"
                         "輪廓；none 是真三角網格，最精確但實測只有半速")
    ap.add_argument("--debug-parts", action="store_true",
                    help="診斷：定期印出角色根節點、UsdSkel 關節、anim.graph 關節"
                         "與部位碰撞體的位置，判斷動畫是否真的驅動了碰撞體")
    ap.add_argument("--walk-mode", choices=("procedural", "anim_people", "off"),
                    default="procedural",
                    help="角色走路動畫："
                         "procedural=自寫步態寫進骨架（確定性、headless 可靠，預設）；"
                         "anim_people=omni.anim.people 行為腳本（headless 下實測起不來）；"
                         "off=不動")
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
    if args.walk_mode == "anim_people":
        # omni.anim.people 要在開 stage 前啟用，動畫圖才註冊得到。
        # ⚠ omni.kit.scripting 不可少：omni.anim.people 的 GoTo 是靠掛在角色
        #   prim 上的 Python 行為腳本執行的。GUI 預設載入這個擴充，精簡 headless
        #   沒有 —— 少了它角色只會播 idle，日誌會出現
        #   "CharacterManager::Shutdown() called without a prior successful
        #    call to CharacterManager::Initialize()"（2026-09-21 實測）。
        for _e in ("omni.kit.scripting", "omni.anim.people",
                   "omni.anim.graph.core", "omni.anim.graph.bundle",
                   "omni.anim.navigation.core", "isaacsim.replicator.agent.core"):
            enable_extension(_e)
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

    _part_track: dict = {"pts": []}

    def _track_part() -> None:
        """每步記錄 L_forearm **相對角色根節點**的位置。

        ⚠ 用世界座標會被角色沿路徑的位移蓋過去（實測走 6 m 時「行程」變成
        6.457 m，完全測不到擺動）。擺幅必須在角色自身的座標系裡量。
        兩點取樣也不行 —— 會剛好撞上同相位而誤判為沒動。
        """
        from pxr import UsdGeom
        import omni.usd as _ou
        st = _ou.get_context().get_stage()
        cache = UsdGeom.XformCache()
        ch = st.GetPrimAtPath("/World/Characters/Character_10")
        pr = st.GetPrimAtPath("/World/Characters/Character_10/lidar_parts/L_forearm")
        if ch and ch.IsValid() and pr and pr.IsValid():
            t = cache.ComputeRelativeTransform(pr, ch)[0].ExtractTranslation()
            _part_track["pts"].append((t[0], t[1], t[2]))

    def _report_part_range(tag: str) -> None:
        pts = _part_track["pts"]
        if len(pts) < 10:
            return
        import math as _m
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
        span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
        print(f"[debug-parts] {tag}: L_forearm(相對角色) 取樣 {len(pts)} 點  "
              f"行程 x={max(xs)-min(xs):.3f} y={max(ys)-min(ys):.3f} "
              f"z={max(zs)-min(zs):.3f} m  最大 {span:.3f} m", flush=True)
        _part_track["pts"] = []

    def _dump_parts(tag: str) -> None:
        """同時量三條路，定位動畫到底有沒有傳到碰撞體。"""
        from pxr import Usd, UsdGeom, UsdSkel
        import omni.usd as _ou
        st = _ou.get_context().get_stage()
        cache = UsdGeom.XformCache()
        CH = "/World/Characters/Character_10"
        ch = st.GetPrimAtPath(CH)
        if not (ch and ch.IsValid()):
            return
        root = cache.GetLocalToWorldTransform(ch).ExtractTranslation()
        msg = [f"root=({root[0]:+.3f},{root[1]:+.3f})"]

        # (a) 傳統 USD 的 UsdSkel
        sk = None
        for pr in Usd.PrimRange(ch, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if pr.GetTypeName() == "Skeleton":
                sk = pr
                break
        if sk:
            sq = UsdSkel.Cache().GetSkelQuery(UsdSkel.Skeleton(sk))
            xf = sq.ComputeJointSkelTransforms(Usd.TimeCode.Default()) if sq else None
            if xf:
                t = xf[min(20, len(xf) - 1)].ExtractTranslation()
                msg.append(f"UsdSkel[20]=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f})")

        # (b) omni.anim.graph.core
        try:
            import omni.anim.graph.core as _ag
            c = _ag.get_character(CH)
            if c is None:
                msg.append("animGraph=取不到")
            else:
                from pxr import Gf
                names = []
                c.get_joint_names(names)
                if names:
                    j = names[min(20, len(names) - 1)]
                    tt, rr = Gf.Vec3d(), Gf.Quatf()
                    c.get_joint_transform(j, tt, rr)
                    msg.append(f"animGraph[{j}]=({tt[0]:+.3f},{tt[1]:+.3f},{tt[2]:+.3f})")
        except Exception as _e:
            msg.append(f"animGraph 失敗={type(_e).__name__}")

        # (c) 我們建的部位碰撞體 + 座標鏈
        pp = st.GetPrimAtPath(f"{CH}/lidar_parts/L_forearm")
        if pp and pp.IsValid():
            t = cache.GetLocalToWorldTransform(pp).ExtractTranslation()
            msg.append(f"part(L_forearm)=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f})")
            ji_a = pp.GetAttribute("charge:jointIndex")
            if ji_a and sk and ji_a.Get() is not None:
                jidx = int(ji_a.Get())
                jn = list(UsdSkel.Skeleton(sk).GetJointsAttr().Get() or [])
                msg.append(f"joint[{jidx}]={jn[jidx].rsplit('/',1)[-1] if jidx < len(jn) else '?'}")
                if xf and jidx < len(xf):
                    jt = xf[jidx].ExtractTranslation()
                    msg.append(f"jointXf=({jt[0]:+.3f},{jt[1]:+.3f},{jt[2]:+.3f})")
                bd = UsdSkel.Skeleton(sk).GetBindTransformsAttr().Get()
                if bd is not None and jidx < len(bd):
                    bt = bd[jidx].ExtractTranslation()
                    msg.append(f"bind=({bt[0]:+.3f},{bt[1]:+.3f},{bt[2]:+.3f})")
                s2c = cache.ComputeRelativeTransform(sk, ch)[0]
                sc = s2c.ExtractTranslation()
                msg.append(f"skel2char_t=({sc[0]:+.3f},{sc[1]:+.3f},{sc[2]:+.3f}) "
                           f"scale={s2c.GetRow3(0).GetLength():.4f}")
        print(f"[debug-parts] {tag}: " + "  ".join(msg), flush=True)

    _dump_poses("play 之前")

    # People 角色：讓光達打得到會擺動的真人形。
    # 必須在執行期做 —— 角色網格在 CDN 參照底下，離線 usd-core 看不到那些 prim。
    parts_driver = None
    walk_driver = None
    if args.character_mode != "off":
        import sys as _s0
        _s0.path.insert(0, str(Path(__file__).resolve().parent))
        from character_colliders import too_close_to_robot
        from pxr import UsdGeom as _UG
        _st = omni.usd.get_context().get_stage()
        _rb = _st.GetPrimAtPath(
            "/World/charger_rover4_5_0/charger_rover_urdf5/base_footprint")
        _cache = _UG.XformCache()
        _rxy = None
        if _rb and _rb.IsValid():
            _t = _cache.GetLocalToWorldTransform(_rb).ExtractTranslation()
            _rxy = (_t[0], _t[1])

        _root = _st.GetPrimAtPath("/World/Characters")
        _ok, _skipped, _walks = [], [], []
        if _root and _root.IsValid():
            for _c in _root.GetChildren():
                if _c.GetName() == "Biped_Setup":
                    continue
                _t = _cache.GetLocalToWorldTransform(_c).ExtractTranslation()
                if _rxy and too_close_to_robot((_t[0], _t[1]), _rxy):
                    _skipped.append(_c.GetName())      # 無限質量會把車彈飛
                    continue
                _ok.append(str(_c.GetPath()))
                _walks.append((_c.GetName(),
                               [(_t[0], _t[1] + 4.0), (_t[0], _t[1] - 4.0)]))

        # anim_people 路線需要 GoTo 命令與 Stop→Play，否則只站著播 idle。
        # 預設的 procedural 路線不走這裡（自己把步態寫進骨架）。
        if args.walk_mode == "anim_people":
            from anim_people import setup as _anim_setup
            print(f"[run_isaac_sim] 動畫設定：{_anim_setup(_st, walks=_walks)[2]}")

        if args.character_mode == "parts":
            from character_parts import build_character_parts
            _np = _nf = 0
            for _cp in _ok:
                a, b, _ = build_character_parts(_st, _cp,
                                                approximation=args.part_approx)
                _np += a; _nf += b
            print(f"[run_isaac_sim] 逐部位碰撞體：{len(_ok)} 人 / {_np} 部位 / {_nf} 面"
                  + (f"　跳過(太靠近車) {_skipped}" if _skipped else ""))
        else:
            from character_colliders import apply_mesh_colliders
            n_c, n_m, sk = apply_mesh_colliders(_st, "/World/Characters", _rxy)
            print(f"[run_isaac_sim] 整具碰撞體(T-pose)：{n_c} 人 / {n_m} mesh"
                  + (f"　跳過 {sk}" if sk else ""))

    if args.walk_mode == "anim_people":
        from anim_people import restart_for_behavior_scripts
        restart_for_behavior_scripts(sim, app)      # 漏掉這步角色站著不動
        print("[run_isaac_sim] Stop→Play 完成，行為腳本已初始化")
    walk_driver = None
    if args.walk_mode == "procedural" and args.character_mode != "off":
        import ros_graph_spec as _S2
        from character_walk import CharacterWalkDriver
        from build_ros_graph import measure_corridor_floor_top
        _st2 = omni.usd.get_context().get_stage()
        walk_driver = CharacterWalkDriver(
            _st2, _ok, walks=_S2.DEFAULT_CHARACTER_WALKS,
            floor_top=measure_corridor_floor_top(_st2))
        print(f"[run_isaac_sim] 程序化步態：{len(walk_driver)} 人 / "
              f"{walk_driver.segments_driven()} 個擺動關節 / "
              f"{walk_driver.walking()} 人沿路徑移動")
    if args.character_mode == "parts":
        from character_parts import CharacterPartsDriver
        parts_driver = CharacterPartsDriver(omni.usd.get_context().get_stage(), _ok)
        print(f"[run_isaac_sim] 部位驅動器：{len(parts_driver)} 塊碰撞體")

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
            if walk_driver is not None:
                walk_driver.update(sim.current_time)   # 先把步態寫進骨架
            if parts_driver is not None:
                parts_driver.update()                  # 部位再跟著骨架走
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
            if args.debug_parts:
                _track_part()
                if steps in (60, 240, 480, 720):
                    _dump_parts(f"step {steps}")
                    _report_part_range(f"step {steps}")
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
