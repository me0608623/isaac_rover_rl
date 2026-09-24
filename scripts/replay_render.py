#!/usr/bin/env python3
"""第二趟：照第一趟錄下的位姿軌跡回放，用 path tracing 算出論文影片。

為什麼要分兩趟（見 pose_log 模組）：
  * RTX Real-Time 在這個場景算出來每一幀都是純黑，只有 path tracing 有畫面。
  * path tracing 把 RTF 壓到 0.35，邊錄邊導航時 cmd_vel 被釘在 0.060 m/s
    （正常 0.475），200 模擬秒只走 14 m，根本到不了終點。
  → 第一趟正常速度跑導航並寫位姿 CSV；第二趟不跑物理、不跑 ROS，
    照 CSV 把車擺回去，慢慢算圖。

行人不必記錄：程序化步態只是模擬時間的函式，兩趟完全一致。

用法：
    ./replay.sh --pose-log run/pose.csv --out run/frames --scenario mixed
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

DEFAULT_USD = Path(__file__).resolve().parents[1] / "assets" / "3floor_ver_1_ros_fixed.usda"
ROBOT_ROOT = "/World/charger_rover4_5_0"
BASE_LINK = f"{ROBOT_ROOT}/charger_rover_urdf5/base_link"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", type=Path, default=DEFAULT_USD)
    ap.add_argument("--pose-log", required=True)
    ap.add_argument("--crowd-log", default="",
                    help="第一遍記下的行人軌跡。ORCA 行人會因應車的動作閃避，"
                         "這裡**不能重算**（更新頻率與積分順序與第一遍不同，"
                         "軌跡會發散、與 rosbag 裡光達打到的人對不起來），只能照播。")
    ap.add_argument("--out", required=True, help="PNG 序列輸出目錄")
    ap.add_argument("--scenario", default="mixed")
    ap.add_argument("--run-index", type=int, default=0,
                    help="場景變體編號。⚠ 必須與第一遍**完全相同**，"
                         "否則影片裡的障礙與行人跟 rosbag 裡光達打到的對不起來。")
    ap.add_argument("--scene", default="",
                    help="第一遍寫的 scene.json。給了就**照抄**它擺場景（啟用哪些障礙、"
                         "哪些角色、誰走誰站、站的朝向），不自己重算規則。"
                         "有 --run-index 時必給。")
    ap.add_argument("--fps", type=float, default=30.0, help="影片幀率（模擬時間）")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--spp", type=int, default=1)
    ap.add_argument("--max-frames", type=int, default=0, help=">0 時只算這麼多幀（測試用）")
    ap.add_argument("--no-trim", action="store_true",
                    help="不要裁掉頭尾的靜止畫面（預設會裁）")
    ap.add_argument("--tick", choices=("step", "app", "render"), default="step",
                    help="用哪種方式推一個算圖幀。BasicWriter 只在**真的算了一幀**"
                         "時才寫檔，挑錯的話會出現「完成 N 幀」卻一個檔都沒有。")
    args = ap.parse_args()

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from pose_log import (crowd_at, motion_window, parse_crowd_rows,
                          parse_rows, pose_at)
    from scenarios import scenario_config

    scen = scenario_config(args.scenario)
    snap = None
    if args.scene:
        # ⚠⚠ 2026-09-23：兩遍各寫一份「誰該停用」的規則已經走樣 —— 第一遍改成
        #   dynamic 停用站立人物、static 停用站在導航點旁的人，這裡沒跟著改，
        #   dynamic/static 的影片裡會出現導航時根本不在場的人。
        #   改成照抄第一遍拍下的快照，兩遍從此不可能不一致。
        from scene_snapshot import read_snapshot
        snap = read_snapshot(Path(args.scene).parent)
        if snap is None:
            print(f"[replay] ⚠ 找不到 {args.scene}", file=sys.stderr)
            return 2
        if snap.get("version", 1) < 2:
            print(f"[replay] ⚠ {args.scene} 是 v1 快照，缺走動行人的路線，不能回放",
                  file=sys.stderr)
            return 2
        if snap["scenario"] != scen.name:
            print(f"[replay] ⚠ 快照情境 {snap['scenario']} 與 --scenario {scen.name} 不同",
                  file=sys.stderr)
            return 2
    elif args.run_index >= 1:
        print("[replay] ⚠ 有 --run-index 卻沒給 --scene —— 不要自己重算場景，"
              "那會跟第一遍不一致（見 scene_snapshot 說明）", file=sys.stderr)
        return 2
    samples = parse_rows(Path(args.pose_log).read_text().splitlines())
    if len(samples) < 2:
        print(f"[replay] 位姿軌跡只有 {len(samples)} 筆，無法回放", file=sys.stderr)
        return 2
    crowd_rows = []
    if args.crowd_log and Path(args.crowd_log).exists():
        crowd_rows = parse_crowd_rows(Path(args.crowd_log).read_text().splitlines())
        print(f"[replay] 行人軌跡 {len(crowd_rows)} 筆 / "
              f"{len({r.name for r in crowd_rows})} 人", flush=True)

    t0, t1 = samples[0].t, samples[-1].t
    if not args.no_trim:
        # 第一趟是「先開 Isaac、再開 ROS、再發初始位姿」，車真正起步大概在
        # 模擬時間第 55 秒。整段照算的話有四成的 path tracing 是白算的，
        # 而且每支影片開頭都是一分鐘的靜止畫面。
        w0, w1 = motion_window(samples)
        print(f"[replay] 裁掉靜止頭尾：{t0:.1f}~{t1:.1f} → {w0:.1f}~{w1:.1f} s",
              flush=True)
        t0, t1 = w0, w1
    n_frames = int((t1 - t0) * args.fps)
    if args.max_frames > 0:
        n_frames = min(n_frames, args.max_frames)
    print(f"[replay] 軌跡 {len(samples)} 筆　模擬時間 {t0:.2f}~{t1:.2f} s "
          f"→ {n_frames} 幀 @ {args.fps} fps", flush=True)

    sys.argv = sys.argv[:1]
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "renderer": "RayTracedLighting",
                         "width": 128, "height": 128})
    app.update()

    import carb
    import omni.usd
    from isaacsim.core.utils.stage import is_stage_loading
    omni.usd.get_context().open_stage(str(args.usd))
    while is_stage_loading():
        app.update()

    from pxr import Gf, UsdGeom
    import omni.replicator.core  # noqa: F401  （要在 SimulationContext 之前）
    from isaacsim.core.api import SimulationContext

    st = omni.usd.get_context().get_stage()
    from build_ros_graph import measure_corridor_floor_top
    from camera_recorder import CameraRecorder
    from character_walk import CharacterWalkDriver
    import ros_graph_spec as S

    floor = measure_corridor_floor_top(st)

    # ── 障礙：照快照開關 ───────────────────────────────────────────────
    want = ({o["name"] for o in snap["obstacles"]} if snap is not None else None)
    obs_root = st.GetPrimAtPath("/World/SimObstacles")
    if obs_root and obs_root.IsValid():
        n_on = 0
        for o in obs_root.GetChildren():
            on = (o.GetName() in want) if want is not None else scen.obstacles_enabled
            o.SetActive(on)
            n_on += 1 if on else 0
        print(f"[replay] 障礙 {n_on} 個啟用"
              + (f"（照第一遍的快照，{snap.get('route') or '?'} 路線 run{snap['run_index']}）"
                 if snap is not None else "（無快照、預設場景）"), flush=True)

    # ── 角色：照快照決定誰在場、誰走、誰擺在哪 ─────────────────────────
    #   不再重算 too_close_to_robot / routing 點 / 疏密 —— 那些規則只在第一遍
    #   跑一次，結果記在快照裡。
    chars = []
    croot = st.GetPrimAtPath("/World/Characters")
    if snap is not None:
        from scene_snapshot import placed_standing, walks_of
        active = {c["name"] for c in snap["characters"]}
        walks = walks_of(snap)
        placements = placed_standing(snap)
    else:
        active = None
        walks = list(S.DEFAULT_CHARACTER_WALKS) if scen.walks_enabled else []
        placements = []
    if croot and croot.IsValid():
        for c in croot.GetChildren():
            if c.GetName() == "Biped_Setup":
                continue
            on = active is None or c.GetName() in active
            c.SetActive(on)
            if on:
                chars.append(str(c.GetPath()))
    if active is not None:
        missing = active - {cp.rsplit("/", 1)[-1] for cp in chars}
        if missing:
            print(f"[replay] ⚠ 快照裡的角色在 stage 找不到：{sorted(missing)}", flush=True)

    walk_driver = CharacterWalkDriver(st, chars, walks=walks, floor_top=floor)
    n_place = 0
    for name, mx, my, yaw in placements:
        if walk_driver.place_standing(name, (mx, my), yaw):
            n_place += 1
    n_ground = walk_driver.ground_standing()
    print(f"[replay] 站立人物擺位 {n_place} 個　腳底對地 {n_ground} 個", flush=True)
    print(f"[replay] 情境 {scen.name}：角色 {len(walk_driver)} 人 / "
          f"{walk_driver.walking()} 人走動　靜態障礙 "
          f"{'啟用' if scen.obstacles_enabled else '停用'}", flush=True)

    # ⚠ 物理**必須**是 play 狀態，否則 BasicWriter 一個檔都不寫
    #   （2026-09-22 實測：sim.render() / app.update() / 未 play 的 sim.step()
    #    都會印出「完成 N 幀」卻產生 0 個檔）。
    #   但物理一跑，PhysX 就擁有車體各 link 的世界位姿，改 USD 沒有用 ——
    #   所以改用 Isaac 的 articulation API 每幀把車瞬移過去，
    #   並且**把重力關掉**，讓瞬移之後不會在下一個物理步被拉走。
    sim = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0,
                            stage_units_in_meters=1.0)

    from pxr import UsdPhysics
    for pr in st.Traverse():
        if pr.IsA(UsdPhysics.Scene):
            UsdPhysics.Scene(pr).CreateGravityMagnitudeAttr(0.0)
            print(f"[replay] 已關閉重力：{pr.GetPath()}", flush=True)

    art_path = None
    for pr in st.Traverse():
        if pr.HasAPI(UsdPhysics.ArticulationRootAPI):
            art_path = str(pr.GetPath())
            break
    if art_path is None:
        print("[replay] 找不到 articulation root", file=sys.stderr)
        return 2
    print(f"[replay] articulation root = {art_path}", flush=True)

    sim.initialize_physics()
    sim.play()
    sim.step(render=False)

    import numpy as np
    from isaacsim.core.prims import SingleArticulation
    art = SingleArticulation(prim_path=art_path, name="replay_robot")
    art.initialize()
    bl = st.GetPrimAtPath(BASE_LINK)

    sett = carb.settings.get_settings()
    sett.set("/rtx/pathtracing/spp", int(args.spp))
    sett.set("/rtx/pathtracing/totalSpp", int(args.spp))
    sett.set("/rtx/pathtracing/optixDenoiser/enabled", True)
    sett.set("/rtx/rendermode", "PathTracing")

    rec = CameraRecorder(st, args.out, args.width, args.height, 1)
    print(f"[replay] 錄影：{len(rec)} 視角 → {args.out} "
          f"{args.width}x{args.height} spp={args.spp}", flush=True)

    if args.tick == "app":
        def tick():
            app.update()
    elif args.tick == "render":
        def tick():
            sim.render()
    else:
        def tick():
            sim.step(render=True)

    import time as _time
    t_start = _time.perf_counter()
    for i in range(n_frames):
        t = t0 + i / args.fps
        p = pose_at(samples, t)
        # ⚠ 有第一遍的行人軌跡就照播，不要重算 ORCA。
        walk_driver.update(t, poses=(crowd_at(crowd_rows, t) if crowd_rows else None))

        art.set_world_pose(position=np.array(p.pos, dtype=np.float32),
                           orientation=np.array(p.quat, dtype=np.float32))
        # SingleArticulation 沒有 set_velocities；線速度/角速度分開設。
        art.set_linear_velocity(np.zeros(3, dtype=np.float32))
        art.set_angular_velocity(np.zeros(3, dtype=np.float32))

        yaw = math.atan2(2.0 * (p.quat[0] * p.quat[3] + p.quat[1] * p.quat[2]),
                         1.0 - 2.0 * (p.quat[2] ** 2 + p.quat[3] ** 2))
        rec.update_poses((p.pos[0], p.pos[1]), yaw, floor)
        rec.step(t)
        tick()

        if i == 0:
            # 驗證瞬移真的把 base_link 放到目標位置 —— articulation 的
            # root link 未必就是 base_link，有固定偏移的話這裡會看得出來。
            _w = UsdGeom.XformCache().GetLocalToWorldTransform(bl).ExtractTranslation()
            _err = max(abs(_w[k] - p.pos[k]) for k in range(3))
            print(f"[replay] 第一幀 base_link 目標 "
                  f"({p.pos[0]:.3f},{p.pos[1]:.3f},{p.pos[2]:.3f}) 實際 "
                  f"({_w[0]:.3f},{_w[1]:.3f},{_w[2]:.3f})  最大誤差 {_err:.4f} m",
                  flush=True)
        if i % 150 == 0 or i == n_frames - 1:
            el = _time.perf_counter() - t_start
            rate = (i + 1) / el if el > 0 else 0.0
            eta = (n_frames - i - 1) / rate if rate > 0 else 0.0
            print(f"[replay] {i+1}/{n_frames} 幀  模擬時間 {t:7.2f} s  "
                  f"{rate:4.1f} 幀/s  剩 {eta/60:5.1f} 分", flush=True)

    print(f"[replay] 完成：{rec.frames_written} 幀/視角", flush=True)
    print(f"[replay] 相機避牆拉近幀數：{getattr(rec, 'pulled_frames', {})}", flush=True)
    rec.close()
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
