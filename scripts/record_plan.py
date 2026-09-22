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

#: 三個視角，順序固定（與 sim_cameras.CAMERAS 對應）。
CAMERAS_IN_PLAN: tuple[str, ...] = ("topdown", "chase", "oblique")


def run_tag(scenario: str, run_index: int) -> str:
    """一趟來回的唯一標籤。

    零補位是刻意的：不補位的話 ``ls`` 會把 run10 排在 run2 前面，
    事後對剪影片與 bag 時很容易拿錯。
    """
    return f"{scenario}_run{run_index:02d}"


@dataclass(frozen=True)
class RunSpec:
    """一趟來回要錄的東西。"""

    scenario: str
    run_index: int
    root: Path

    @property
    def tag(self) -> str:
        return run_tag(self.scenario, self.run_index)

    @property
    def run_dir(self) -> Path:
        return self.root / self.tag

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


def build_plan(runs_per_scenario: int, root) -> tuple[RunSpec, ...]:
    """依情境分組產生整批計畫。

    情境放外層、趟次放內層：同一個情境的四趟連著跑，
    中途若要停手，至少會有完整的一組情境可用。
    """
    if runs_per_scenario < 1:
        raise ValueError(f"每個情境至少要一趟，收到 {runs_per_scenario}")
    root = Path(root)
    return tuple(
        RunSpec(scenario=s, run_index=i, root=root)
        for s in SCENARIO_NAMES
        for i in range(1, runs_per_scenario + 1)
    )
