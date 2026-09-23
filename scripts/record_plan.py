"""論文錄影批次的計畫表（純資料，不動 Isaac、不動檔案系統）。

使用者要的是「3 情境 × 4 趟來回 × 3 視角 = 36 段」，而且**影片與 rosbag 命名
要同步**，之後才能用 RViz 播 bag 去對剪。

所以命名的唯一真相在這裡：一趟來回一個 tag，影片、rosbag、逐時刻 CSV、
run.json 全部掛在同一個 tag 底下。

目錄長相：
    <root>/<tag>/
        frames/{topdown,chase,oblique}/rgb_XXXX.png   ← Isaac 直接寫
        frames/frame_times.csv                        ← 幀號 ↔ 模擬時間
        video/<tag>_{topdown,chase,oblique}.mp4       ← ffmpeg 編出來
        bag/                                          ← ros2 bag record
        nav/                                          ← 逐時刻 CSV
        run.json                                      ← 這趟的參數與結果
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from scenarios import SCENARIO_NAMES

from run_layout import run_dir_for

#: 要錄的 RL 模型 profile。
#:
#: 取自車端 `deploy_select.sh` 的互動選單（= models/*.ts 排序後扣掉 HIDE_TS），
#: **排除第一項** sa4_e2e_fs4_cleanppo_89600.ts —— 那顆在 sim 端沒有對應的
#: profile（沒有配套的 policy / preprocessor yaml），而這些模型的觀測契約不同，
#: 只換 model_path 而沿用別人的 yaml 會安靜地算出垃圾動作。
#: 2026-09-22 使用者指定：選單上除第一項外的其他模型全部錄。
DEFAULT_MODELS: tuple[str, ...] = ("sa4r2", "sa4r3", "sa5r2")

#: 三個視角，順序固定（與 sim_cameras.CAMERAS 對應）。
CAMERAS_IN_PLAN: tuple[str, ...] = ("topdown", "chase", "oblique")


def run_tag(model: str, route: str, scenario: str, run_index: int) -> str:
    """一趟來回的唯一標籤，例如 ``sa4r2_c27_static_run01``。

    模型放最前面：同一個模型的趟會排在一起，`ls` 出來就是分組的。
    ⚠ 路線一定要在標籤裡：2026-09-23 加入第二條路線，兩條路線同名的
      ``sa4r2_static_run01`` 會寫進同一個資料夾互相覆蓋。
    零補位是刻意的：不補位的話 ``ls`` 會把 run10 排在 run2 前面，
    事後對剪影片與 bag 時很容易拿錯。
    """
    if not route:
        raise ValueError("路線是空的 —— 兩條路線的趟會撞名")
    return f"{model}_{route}_{scenario}_run{run_index:02d}"


@dataclass(frozen=True)
class RunSpec:
    """一趟來回要錄的東西。"""

    model: str
    scenario: str
    run_index: int
    root: Path
    route: str = "c27"

    @property
    def tag(self) -> str:
        return run_tag(self.model, self.route, self.scenario, self.run_index)

    @property
    def run_dir(self) -> Path:
        return run_dir_for(self.root, self.model, self.tag)

    @property
    def frames_dir(self) -> Path:
        return self.run_dir / "frames"

    @property
    def video_dir(self) -> Path:
        return self.run_dir / "video"

    @property
    def bag_dir(self) -> Path:
        return self.run_dir / "bag"

    @property
    def nav_dir(self) -> Path:
        return self.run_dir / "nav"

    @property
    def meta_path(self) -> Path:
        return self.run_dir / "run.json"

    def video_path(self, camera: str) -> Path:
        return self.video_dir / f"{self.tag}_{camera}.mp4"


def build_plan(runs_per_scenario: int, root, models=DEFAULT_MODELS,
               routes=None) -> tuple[RunSpec, ...]:
    """依 路線 → 模型 → 情境 → 趟次 四層產生整批計畫。

    路線放最外層：主路線（c27）整批先跑完。使用者說「主要以 c28→c27 為主」，
    中途若要停手，至少主路線是完整的。模型次之：同一個模型的整組連著跑完，
    不會三個模型各缺一半。
    """
    import ros_graph_spec as S
    routes = tuple(routes) if routes is not None else S.ROUTE_ORDER
    if not routes:
        raise ValueError("路線清單是空的")
    for rk in routes:
        S.route(rk)                       # 打錯路線名要在開跑前就炸
    if runs_per_scenario < 1:
        raise ValueError(f"每個情境至少要一趟，收到 {runs_per_scenario}")
    if not models:
        raise ValueError("模型清單是空的")
    root = Path(root)
    return tuple(
        RunSpec(model=m, scenario=s, run_index=i, root=root, route=rk)
        for rk in routes
        for m in models
        for s in SCENARIO_NAMES
        for i in range(1, runs_per_scenario + 1)
    )
