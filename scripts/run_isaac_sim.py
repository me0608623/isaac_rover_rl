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
import math
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
    ap.add_argument("--record-dir", default="",
                    help="錄三視角影片到這個目錄（PNG 序列，之後用 ffmpeg 編碼）。"
                         "⚠ 算圖會拖慢模擬；policy 跑在牆鐘上，RTF<1 會讓每個"
                         "模擬秒拿到比實車更多次決策，錄下的行為不代表真實表現。")
    ap.add_argument("--record-width", type=int, default=1280)
    ap.add_argument("--record-height", type=int, default=720)
    ap.add_argument("--run-index", type=int, default=0,
                    help="場景變體編號（1 起算）。>0 時障礙與行人的**數量與位置**"
                         "依 scene_variants.variant() 決定，每一趟都不一樣、"
                         "且數量依序更多。0=沿用 ros_graph_spec 的固定預設場景。")
    ap.add_argument("--crowd-mode", choices=("orca", "path"), default="orca",
                    help="行人怎麼走：orca(預設)=用 RVO2 解 ORCA，會與車和彼此"
                         "互相閃避；path=沿固定折線等速往返（舊行為，不理會車）")
    ap.add_argument("--crowd-log", default="",
                    help="把每個行人的位姿逐幀寫成 CSV。ORCA 下行人會因應車的"
                         "動作閃避，第二遍回放不能重算（更新頻率與積分順序不同，"
                         "軌跡會發散），只能照播。錄影時必給。")
    ap.add_argument("--pose-log", default="",
                    help="把車體的世界位姿逐幀寫成 CSV，供第二趟回放算圖用。"
                         "錄影必須分兩趟：path tracing 會把 RTF 壓到 0.35，"
                         "邊錄邊導航時 cmd_vel 被釘在 0.060 m/s（正常 0.475）"
                         "根本到不了終點。見 pose_log 模組說明。")
    ap.add_argument("--scenario", default="mixed",
                    help="錄影情境：static=只有靜態障礙、行人站著；"
                         "dynamic=只有走動的行人、關掉靜態障礙；"
                         "mixed(預設)=兩者都有（已驗證過的正式組態）")
    ap.add_argument("--record-spp", type=int, default=1,
                    help="錄影用的 path tracing 每幀取樣數。1(預設)配 OptiX "
                         "denoiser 實測就夠乾淨，且每幀 191 ms；4 要 714 ms、"
                         "16 要 3.2 s，畫面亮度卻沒差。")
    ap.add_argument("--record-every", type=int, default=1,
                    help="frame_times.csv 的記帳間隔。⚠ **不會**減少實際輸出的"
                         "張數 —— BasicWriter 掛上 render product 之後每個算圖"
                         "幀都會自己寫檔（見 camera_recorder.step 的說明）。"
                         "設 >1 只會讓 CSV 的幀號與檔名對不上，除非你知道自己"
                         "在做什麼，否則保持 1。")
    ap.add_argument("--colliders-for", choices=("all", "walkers"), default="all",
                    help="哪些角色要有碰撞體。all(預設)=全部；walkers=只給會走路的。"
                         "⚠ 2026-09-21 曾預設 walkers，理由是「角色點雲污染 NDT」——"
                         "該假設已被對照實驗推翻，見下方 _colliders_note。")
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

    # 情境設定要在開 Isaac 之前就查好 —— 名字打錯要馬上報錯，
    # 不要等 Isaac 起來兩分鐘之後才發現。
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from scenarios import scenario_config
    scen = scenario_config(args.scenario)

    if not args.usd.exists():
        print(f"[run_isaac_sim] 找不到 USD: {args.usd}", file=sys.stderr)
        return 2

    # Isaac 會把它不認得的 CLI 參數原封不動轉給底層的 Kit
    # （啟動時會印 "Passing the following args to the base kit application"）。
    # 參數在上面已經 parse 完了，別讓自家旗標流進 Kit 的設定解析。
    sys.argv = sys.argv[:1]

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
    # ⚠ omni.replicator.core 必須在**開 stage、起物理之前**就 import。
    #   import 這個模組會順帶把 replicator 擴充與 SDG 算圖管線叫起來；
    #   等到 sim.play() 之後才第一次 import（例如在 CameraRecorder 裡），
    #   之後建立的 render product 算出來會是**全黑**。
    #   2026-09-22 二分法實測：同一份 USD、同一段 A/B 程式碼、
    #   carb 設定逐項比對完全相同，差別只在這行的位置 ——
    #   先 import 亮度 47~54，後 import 一律 0.00。
    app.update()

    import omni.usd
    from isaacsim.core.utils.stage import is_stage_loading

    print(f"[run_isaac_sim] 開啟 {args.usd}")
    omni.usd.get_context().open_stage(str(args.usd))
    while is_stage_loading():
        app.update()
    print("[run_isaac_sim] stage 載入完成")

    # 場景變體：USD 裡寫了**所有** run 變體的障礙，這裡只開這一趟要用的。
    _sv = None
    if args.run_index >= 1:
        from scene_variants import variant as _variant
        _sv = _variant(args.run_index)
    _want = ({o.name for o in _sv.obstacles} if _sv is not None else None)

    # 情境：關掉靜態障礙就整個 prim 停用（SetActive(False) 會同時從算圖與
    # 物理中移除，不必分別處理可見性與碰撞體）。
    _obs_root = omni.usd.get_context().get_stage().GetPrimAtPath(
        "/World/SimObstacles")
    if _obs_root and _obs_root.IsValid():
        _n_on = _n_all = 0
        for _o in _obs_root.GetChildren():
            _n_all += 1
            _on = scen.obstacles_enabled and (_want is None
                                              or _o.GetName() in _want)
            _o.SetActive(_on)
            _n_on += 1 if _on else 0
        print(f"[run_isaac_sim] 情境 {scen.name}"
              + (f"／變體 run{args.run_index}" if _sv else "")
              + f"：靜態障礙 {_n_on}/{_n_all} 啟用"
              f"　行人走動 {'開' if scen.walks_enabled else '關'}")

    from pxr import Gf

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
        from character_colliders import (nearest_routing_node, thin_by_spacing,
                                         too_close_to_robot,
                                         too_close_to_routing_node)
        import ros_graph_spec as _S0
        _stations = _S0.read_station_nodes(_S0.ROUTING_STATION_JSON)
        _variant_walks = (_sv.walks if _sv is not None
                          else _S0.DEFAULT_CHARACTER_WALKS)
        _walk_names0 = {w.name for w in _variant_walks}
        # 變體指定了名單就照名單，其餘角色整個不出現（讓「生成位置」跟著變）。
        _allowed = ((_walk_names0 | {p_.name for p_ in _sv.standing})
                    if _sv is not None else None)
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
        # 「太靠近車」的角色**整個停用**，不是只跳過碰撞體。
        #   理由：不給碰撞體是因為 kinematic 角色等於無限質量，會把車彈飛；
        #   但這種角色對感測是完全透明的（PhysX 光達只打碰撞體），
        #   留著只會在車後視角裡變成一個貼在鏡頭前、擋住整台車的人 ——
        #   而且因為不在驅動清單裡，他會一直停在綁定姿勢（T-pose）。
        #   SetActive(False) 同時從算圖與物理移除，對光達/NDT 沒有任何影響。
        _all_chars = []
        _on_node = []
        if _root and _root.IsValid():
            for _c in _root.GetChildren():
                if _c.GetName() == "Biped_Setup":
                    continue
                if _allowed is not None and _c.GetName() not in _allowed:
                    _c.SetActive(False)
                    continue
                _t = _cache.GetLocalToWorldTransform(_c).ExtractTranslation()
                if _rxy and too_close_to_robot((_t[0], _t[1]), _rxy):
                    _skipped.append(_c.GetName())
                    _c.SetActive(False)
                    continue
                # 站著的角色不得壓在 routing 站點上（會擋住導航目標）。
                # 會走的角色**在真的會走的時候**不適用 —— 它們的路線本來就沿
                # 走廊、必然經過站點，只是路過不是佔住。
                #
                # ⚠⚠ 2026-09-23：原本無條件豁免 `_walk_names0`，但 `static`
                #   情境 walks_enabled=False，那些角色是**停在 USD 原位不動的**，
                #   等於靜態行人。實測 Character_11 停在路線站 c26 旁 0.709 m、
                #   Character_13 停在新終點 c27 旁 0.995 m —— 抵達半徑就是 1.0 m，
                #   static 那幾趟會到不了終點。所以豁免要看 walks_enabled。
                _will_walk = scen.walks_enabled and _c.GetName() in _walk_names0
                if not _will_walk:
                    _mx, _my, _ = _S0.world_to_map(_t[0], _t[1], 0.0)
                    if too_close_to_routing_node((_mx, _my), _stations):
                        _nn, _nd = nearest_routing_node((_mx, _my), _stations)
                        _on_node.append(f"{_c.GetName()}@{_nn}({_nd:.2f}m)")
                        _c.SetActive(False)
                        continue
                _all_chars.append(str(_c.GetPath()))
                _ok.append(str(_c.GetPath()))
                _walks.append((_c.GetName(),
                               [(_t[0], _t[1] + 4.0), (_t[0], _t[1] - 4.0)]))
        if _on_node:
            print(f"[run_isaac_sim] 停用(站在 routing 點位上) {_on_node}")

        # 站立角色擠在一起就挑掉幾個（會走的不算 —— 它們本來就會散開）。
        _stand = [(p_.rsplit("/", 1)[-1], p_) for p_ in _all_chars
                  if p_.rsplit("/", 1)[-1] not in _walk_names0]
        _xy = {}
        for _nm, _p in _stand:
            _tt = _cache.GetLocalToWorldTransform(
                _st.GetPrimAtPath(_p)).ExtractTranslation()
            _xy[_nm] = (_tt[0], _tt[1])
        _kept, _crowded = thin_by_spacing([(n, _xy[n]) for n, _ in _stand])
        if _crowded:
            _drop = set(_crowded)
            for _nm, _p in _stand:
                if _nm in _drop:
                    _st.GetPrimAtPath(_p).SetActive(False)
            _all_chars = [p_ for p_ in _all_chars
                          if p_.rsplit("/", 1)[-1] not in _drop]
            _ok = [p_ for p_ in _ok if p_.rsplit("/", 1)[-1] not in _drop]
            _walks = [w for w in _walks if w[0] not in _drop]
            print(f"[run_isaac_sim] 停用(站太近、降低密度) {_crowded}")

        # ⚠⚠ 這裡曾經預設只給「會走路的角色」碰撞體，理由是「角色點雲污染 NDT」。
        #   **該假設已被對照實驗推翻**（2026-09-21，車靜止 60 s）：
        #
        #       角色數    傾角最大   水平漂移    後半段−前半段
        #          0       1.65°     0.001 m      +0.00°
        #          5       1.89°     0.017 m      -0.04°
        #         19       2.08°     0.022 m      -0.04°
        #
        #   0→19 個角色只差 0.43°，三組皆無惡化趨勢；對照導航失敗時的 18.76°
        #   （且呈 2°→8°→18° 單調爬升），角色的影響連零頭都不到。
        #
        #   原本的證據（「即時點雲與地圖重疊率 100%→48.7%」）是**錯的量測**：
        #   /filtered_points 在感測器座標系、/ndt_points_map 在 map 座標系，
        #   比對不同座標系的體素得到的數字沒有意義。
        #
        #   更根本的推理錯誤：**實車也有行人且定位正常**，所以行人不可能是元凶。
        #   模擬出問題而實機正常時，要找的是**兩者不一樣**的東西。
        #
        #   保留 walkers 選項只為了重現當時的條件，預設已還原為 all。
        import ros_graph_spec as _S1
        _walk_names = {w.name for w in _S1.DEFAULT_CHARACTER_WALKS}
        if args.colliders_for == "walkers":
            _all_ok = list(_ok)
            _ok = [p for p in _ok if p.rsplit("/", 1)[-1] in _walk_names]
            print(f"[run_isaac_sim] 碰撞體只給會走的角色：{len(_ok)} / {len(_all_ok)} 人"
                  f"（站著的不給，避免污染 NDT）")

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
                  + (f"　停用(太靠近車、只會擋鏡頭) {_skipped}" if _skipped else ""))
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
            _st2, _all_chars,
            walks=(_sv.walks if _sv is not None else _S2.DEFAULT_CHARACTER_WALKS)
                  if scen.walks_enabled else (),
            floor_top=measure_corridor_floor_top(_st2))
        # 站立的人物擺到「人形障礙」的位置（圓柱不可見、只當碰撞體）。
        #
        # ⚠⚠ 2026-09-23：障礙關閉時（dynamic）要把這些角色**整個停用**，
        #   不是擺位。原本不看 scen.obstacles_enabled，於是 dynamic 印
        #   「靜態障礙 0/18 啟用」卻仍有 3~5 個站著不動的人形障礙 ——
        #   dynamic 與 mixed 的靜態內容因此幾乎相同（第1、2趟完全一樣），
        #   「有/沒有靜態障礙」的對比根本不成立。
        #   只是不擺位也不行：那些角色會留在 USD 原位，而 Character_10~13、19
        #   的原位就在走廊中線上，照樣擋路。必須 SetActive(False)。
        _n_place = 0
        _n_off = 0
        _standing_now = []
        if _sv is not None:
            for _p in _sv.standing:
                if not scen.obstacles_enabled:
                    _pp = _st2.GetPrimAtPath(f"/World/Characters/{_p.name}")
                    if _pp and _pp.IsValid():
                        _pp.SetActive(False)
                        _n_off += 1
                    continue
                if walk_driver.place_standing(_p.name, (_p.map_x, _p.map_y), _p.yaw):
                    _n_place += 1
                    _standing_now.append((_p.map_x, _p.map_y))
        if _n_off:
            print(f"[run_isaac_sim] 停用(障礙關閉，站立人物不該存在) {_n_off} 個")
        # 其餘站著的人降到地板（USD 原本懸空 17~20 cm）
        _n_ground = walk_driver.ground_standing()
        print(f"[run_isaac_sim] 程序化步態：{len(walk_driver)} 人 / "
              f"{walk_driver.segments_driven()} 個擺動關節 / "
              f"{walk_driver.walking()} 人沿路徑移動"
              f"　站立人物擺位 {_n_place} 個　腳底對地 {_n_ground} 個")
    crowd = None
    if (args.crowd_mode == "orca" and walk_driver is not None
            and scen.walks_enabled):
        from orca_crowd import OrcaCrowd, ccw_rect, oriented_rect
        # ⚠⚠ 2026-09-23 使用者回報「走動行人會穿過站著的人」。實測 dynamic 那
        #   12 趟最近距離低到 0.001 m、37~60% 的幀都在重疊。原因：這裡只在
        #   `obstacles_enabled` 時才餵障礙給 ORCA，dynamic 餵 0 個，
        #   於是走動行人根本不知道那些站著的人存在。
        #   判準改成「**場上實際有什麼**」：站立人物擺了就餵（_standing_now），
        #   不管圓柱有沒有啟用。
        _obs = []
        _obs_at = []                     # 每個矩形的中心，用來去重
        if scen.obstacles_enabled:
            for _o in (_sv.obstacles if _sv is not None else _S2.DEFAULT_OBSTACLES):
                if _o.kind == "prop":
                    # 道具：bbox 中心就是 (map_x, map_y)，長邊沿走廊 → 要旋轉
                    _obs.append(oriented_rect(
                        _o.map_x, _o.map_y, _o.size_x / 2.0 + 0.15,
                        _o.size_y / 2.0 + 0.15, math.radians(_o.yaw_deg)))
                else:
                    _hx = (_o.size_x / 2.0 if _o.kind == "box" else 0.25)
                    _hy = (_o.size_y / 2.0 if _o.kind == "box" else 0.25)
                    _obs.append(ccw_rect(_o.map_x, _o.map_y, _hx + 0.15, _hy + 0.15))
                _obs_at.append((_o.map_x, _o.map_y))
        # 保險：站立人物一律要在 ORCA 的世界裡，即使某天障礙圓柱與站立人物
        # 不再一對一。去重要比**中心**，不是矩形的某個頂點。
        for _sx, _sy in _standing_now:
            if not any(math.hypot(_sx - _cx, _sy - _cy) < 0.30
                       for _cx, _cy in _obs_at):
                _obs.append(ccw_rect(_sx, _sy, 0.40, 0.40))
                _obs_at.append((_sx, _sy))
        # 走廊兩側牆：擋住行人被 ORCA 推出走廊（走廊約 map y∈[3,8]）
        _walls = (ccw_rect(-10.0, 2.3, 14.0, 0.3),
                  ccw_rect(-10.0, 8.7, 14.0, 0.3))
        crowd = OrcaCrowd([w for w in (_sv.walks if _sv is not None
                                       else _S2.DEFAULT_CHARACTER_WALKS)
                           if w.name in {c.rsplit("/", 1)[-1] for c in _all_chars}],
                          time_step=1.0 / args.physics_hz,
                          obstacles=_obs, wall_bands=_walls)
        print(f"[run_isaac_sim] ORCA 行人：{len(crowd)} 人　"
              f"靜態障礙 {len(_obs)} 個　牆 {len(_walls)} 段")

    if args.character_mode == "parts":
        from character_parts import CharacterPartsDriver
        parts_driver = CharacterPartsDriver(omni.usd.get_context().get_stage(), _ok)
        print(f"[run_isaac_sim] 部位驅動器：{len(parts_driver)} 塊碰撞體")

    recorder = None
    if args.record_dir:
        from camera_recorder import CameraRecorder
        from build_ros_graph import measure_corridor_floor_top as _mcft
        _st3 = omni.usd.get_context().get_stage()
        # ⚠⚠ 錄影一定要用 **path tracing**。RTX Real-Time
        #   （rendermode="RaytracedLighting"，Isaac 的預設）在這個場景算出來
        #   **每一幀都是純 0**，跟光源、AA 模式、相機位置都無關：
        #     2026-09-22 逐幀量測（1280x720、三視角、每組獨立輸出目錄）
        #       rt   aa=3(DLSS) / 0(none) / 1(TAA) / 2(FXAA) → 全部 0
        #       pt   spp=1  → 每幀 190，191 ms/幀
        #       pt   spp=4  → 每幀 191，714 ms/幀
        #       pt   spp=16 → 每幀 191，3222 ms/幀
        #   spp 再高畫面亮度不變，所以 spp=1 + OptiX denoiser 就夠。
        #
        #   ⚠ 誤判紀錄：先前以為「重設 /rtx/rendermode 就會亮」，那是**量測
        #     假象** —— 我取每組的最後 4 幀平均，而 path tracing 是累積式的，
        #     40 幀裡只有最後一幀是完成品(190)，其餘 39 幀是 0，平均成 47.6
        #     看起來像亮了。逐幀列印才看得到真相。量測方式錯，結論一定錯。
        import carb
        _sett = carb.settings.get_settings()
        _sett.set("/rtx/pathtracing/spp", int(args.record_spp))
        _sett.set("/rtx/pathtracing/totalSpp", int(args.record_spp))
        _sett.set("/rtx/pathtracing/optixDenoiser/enabled", True)
        _sett.set("/rtx/rendermode", "PathTracing")
        recorder = CameraRecorder(_st3, args.record_dir, args.record_width,
                                  args.record_height, args.record_every)
        _rec_floor = _mcft(_st3)
        print(f"[run_isaac_sim] 錄影：{len(recorder)} 視角 → {args.record_dir}"
              f"  {args.record_width}x{args.record_height}"
              f"  每 {args.record_every} tick 一幀")

    crowd_fp = None
    if args.crowd_log:
        from pose_log import CROWD_HEADER as _CH
        Path(args.crowd_log).parent.mkdir(parents=True, exist_ok=True)
        crowd_fp = open(args.crowd_log, "w")
        crowd_fp.write(_CH + "\n")
        print(f"[run_isaac_sim] 行人軌跡 → {args.crowd_log}")

    pose_fp = None
    if args.pose_log:
        from pose_log import HEADER as _POSE_HEADER
        Path(args.pose_log).parent.mkdir(parents=True, exist_ok=True)
        pose_fp = open(args.pose_log, "w")
        pose_fp.write(_POSE_HEADER + "\n")
        print(f"[run_isaac_sim] 位姿軌跡 → {args.pose_log}")

        # 場上實際有什麼 → scene.json（事後分析一律讀它，不重算 variant）。
        # ⚠ 必須在**所有停用/擺位都做完之後**才拍：太靠近車、站在 routing 點上、
        #   擠在一起、障礙關閉這幾種停用都發生在前面。
        try:
            import sys as _s9
            _s9.path.insert(0, str(Path(__file__).resolve().parent))
            import ros_graph_spec as _S9
            from pxr import UsdGeom as _UG9
            from scene_snapshot import build_snapshot, write_snapshot
            _st9 = omni.usd.get_context().get_stage()
            _on_obs = []
            _obs_root9 = _st9.GetPrimAtPath("/World/SimObstacles")
            if _sv is not None and _obs_root9 and _obs_root9.IsValid():
                _by9 = {o.name: o for o in _sv.obstacles}
                for _c9 in _obs_root9.GetChildren():
                    if _c9.IsActive() and _c9.GetName() in _by9:
                        _on_obs.append(_by9[_c9.GetName()])
            _walking9 = ({w.name for w in (_sv.walks if _sv is not None
                                           else _S9.DEFAULT_CHARACTER_WALKS)}
                         if scen.walks_enabled else set())
            _chars9 = []
            _root9 = _st9.GetPrimAtPath("/World/Characters")
            _cache9 = _UG9.XformCache()
            if _root9 and _root9.IsValid():
                for _c9 in _root9.GetChildren():
                    if _c9.GetName() == "Biped_Setup" or not _c9.IsActive():
                        continue
                    _t9 = _cache9.GetLocalToWorldTransform(_c9).ExtractTranslation()
                    _mx9, _my9, _ = _S9.world_to_map(_t9[0], _t9[1], 0.0)
                    _chars9.append((_c9.GetName(), _mx9, _my9,
                                    _c9.GetName() in _walking9))
            _snap_p = write_snapshot(Path(args.pose_log).parent, build_snapshot(
                scen.name, args.run_index, _on_obs, _chars9))
            print(f"[run_isaac_sim] 場景快照 → {_snap_p}"
                  f"（障礙 {len(_on_obs)}、角色 {len(_chars9)}，其中會走 "
                  f"{sum(1 for c in _chars9 if c[3])}）")
        except Exception as _e9:              # 快照失敗不該中斷錄影，但要大聲說
            print(f"[run_isaac_sim] ⚠ 場景快照寫入失敗：{_e9!r}")

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

    _orca_prev: dict = {}
    do_render_prev = [True]      # 行人紀錄與位姿紀錄同頻（見迴圈內註解）
    steps = 0
    t_start = time.perf_counter()
    t_report = t_start
    sim_report = sim.current_time
    try:
        while app.is_running():
            if walk_driver is not None:
                _cp = None
                if crowd is not None:
                    from pxr import UsdGeom as _UG0
                    import ros_graph_spec as _S9
                    _rp = omni.usd.get_context().get_stage().GetPrimAtPath(
                        "/World/charger_rover4_5_0/charger_rover_urdf5/base_link")
                    _rm = _UG0.XformCache().GetLocalToWorldTransform(_rp)
                    _rt = _rm.ExtractTranslation()
                    _rmx, _rmy, _ = _S9.world_to_map(_rt[0], _rt[1], 0.0)
                    _prev = _orca_prev.get("xy")
                    _rv = ((0.0, 0.0) if _prev is None else
                           ((_rmx - _prev[0]) * args.physics_hz,
                            (_rmy - _prev[1]) * args.physics_hz))
                    _orca_prev["xy"] = (_rmx, _rmy)
                    _cp = crowd.step((_rmx, _rmy), _rv)
                    if crowd_fp is not None and do_render_prev[0]:
                        from pose_log import (CrowdSample as _CS,
                                              format_crowd_row as _fcr)
                        for _n, _v in _cp.items():
                            crowd_fp.write(_fcr(_CS(
                                sim.current_time, _n, _v[0], _v[1],
                                _v[2] if _v[2] is not None else
                                walk_driver.last_yaw(_n), _v[3], _v[4])) + "\n")
                walk_driver.update(sim.current_time, poses=_cp)
            if parts_driver is not None:
                parts_driver.update()                  # 部位再跟著骨架走
            # ⚠ do_render 必須先算 —— 錄影區塊要用它。
            #   2026-09-22 曾把錄影放在這行之前，第一次迭代就 NameError，
            #   直接跳到 finally，錄出 0 幀而且沒有明顯錯誤訊息。
            do_render = render_every > 0 and steps % render_every == 0
            if recorder is not None and do_render:
                # ⚠ 錄影失敗不該讓整個模擬停擺，但**必須大聲**：
                #   2026-09-22 兩次錄出 0 幀，因為例外被 finally 吞掉，
                #   log 裡只看到「錄影結束：0 幀」，完全查不出原因。
                try:
                    from pxr import UsdGeom as _UG2
                    _rb2 = omni.usd.get_context().get_stage().GetPrimAtPath(
                        "/World/charger_rover4_5_0/charger_rover_urdf5/base_footprint")
                    if _rb2 and _rb2.IsValid():
                        _m2 = _UG2.XformCache().GetLocalToWorldTransform(_rb2)
                        _t2 = _m2.ExtractTranslation()
                        _r2 = _m2.ExtractRotation()
                        _yaw2 = math.radians(
                            _r2.Decompose(Gf.Vec3d(0, 0, 1), Gf.Vec3d(0, 1, 0),
                                          Gf.Vec3d(1, 0, 0))[0])
                        recorder.update_poses((_t2[0], _t2[1]), _yaw2, _rec_floor)
                    recorder.step(sim.current_time)
                except Exception:
                    import traceback
                    print("[run_isaac_sim] ⚠ 錄影失敗，停止錄影並繼續模擬：",
                          flush=True)
                    traceback.print_exc()
                    recorder = None
            if pose_fp is not None and do_render:
                # 記 base_link（車體本身）的世界位姿。回放時把整台車的根節點
                # 挪到「base_link 落在這個位姿」的地方即可。
                from pxr import UsdGeom as _UG3
                from pose_log import PoseSample as _PS, format_row as _fr
                _pb = omni.usd.get_context().get_stage().GetPrimAtPath(
                    "/World/charger_rover4_5_0/charger_rover_urdf5/base_link")
                if _pb and _pb.IsValid():
                    _mm = _UG3.XformCache().GetLocalToWorldTransform(_pb)
                    _tt = _mm.ExtractTranslation()
                    _qq = _mm.ExtractRotationQuat()
                    _im = _qq.GetImaginary()
                    pose_fp.write(_fr(_PS(
                        sim.current_time, (_tt[0], _tt[1], _tt[2]),
                        (_qq.GetReal(), _im[0], _im[1], _im[2]))) + "\n")
            do_render_prev[0] = do_render
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
        if crowd_fp is not None:
            crowd_fp.close()
            print(f"[run_isaac_sim] 行人軌跡已寫入 {args.crowd_log}")
        if pose_fp is not None:
            pose_fp.close()
            print(f"[run_isaac_sim] 位姿軌跡已寫入 {args.pose_log}")
        if recorder is not None:
            print(f"[run_isaac_sim] 錄影結束：{recorder.frames_written} 幀/視角")
            recorder.close()
        sim.stop()
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
