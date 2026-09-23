"""中文索引樹的測試。"""

from __future__ import annotations

import json
import os

from make_browse_tree import BROWSE_DIRNAME, MAIN_DIRNAME, RAW_DIRNAME, build


def _mk_run(root, tag, scenario, idx, cams=("topdown", "chase", "oblique"),
            obstacles=6, chars=13, walking=8, standing=5, crowd_mode="orca"):
    d = root / tag
    (d / "video").mkdir(parents=True)
    (d / "run.json").write_text(json.dumps(
        {"tag": tag, "scenario": scenario, "run_index": idx,
         "crowd_mode": crowd_mode}))
    (d / "isaac_nav.log").write_text(
        f"[run_isaac_sim] 情境 {scenario}／變體 run{idx}："
        f"靜態障礙 {obstacles}/18 啟用　行人走動 開\n"
        f"[run_isaac_sim] 程序化步態：{chars} 人 / 104 個擺動關節 / "
        f"{walking} 人沿路徑移動　站立人物擺位 {standing} 個　腳底對地 5 個\n")
    for c in cams:
        (d / "video" / f"{tag}_{c}.mp4").write_text("x")
    return d


def test_build_links_every_video(tmp_path):
    root = tmp_path / "rec"
    root.mkdir()
    _mk_run(root, "sa4r2_mixed_run04", "mixed", 4)
    stats = build(root, {})
    assert stats[MAIN_DIRNAME] == 3
    got = sorted(p.name for p in
                 (root / BROWSE_DIRNAME / MAIN_DIRNAME / "模型sa4r2").iterdir())
    assert got == ["混合_ORCA互動_第4趟_靜6動8_俯視.mp4",
                   "混合_ORCA互動_第4趟_靜6動8_斜前方.mp4",
                   "混合_ORCA互動_第4趟_靜6動8_車後.mp4"]


def test_links_are_relative_and_resolve(tmp_path):
    """★ 絕對路徑的連結在 recordings/ 被搬走或掛到別台之後全部斷掉。"""
    root = tmp_path / "rec"
    root.mkdir()
    _mk_run(root, "sa4r2_static_run01", "static", 1, obstacles=3,
            chars=5, walking=0, standing=3)
    build(root, {})
    link = (root / BROWSE_DIRNAME / MAIN_DIRNAME / "模型sa4r2"
            / "純靜態_不走動_第1趟_靜5動0_俯視.mp4")
    assert link.is_symlink()
    assert not os.path.isabs(os.readlink(link))
    assert link.resolve().read_text() == "x"


def test_raw_data_link_points_at_the_run_directory(tmp_path):
    root = tmp_path / "rec"
    root.mkdir()
    _mk_run(root, "sa4r2_dynamic_run02", "dynamic", 2, obstacles=0,
            chars=8, walking=4, standing=4)
    build(root, {})
    link = (root / BROWSE_DIRNAME / RAW_DIRNAME
            / "模型sa4r2_純動態_ORCA互動_第2趟_靜4動4")
    assert link.resolve().name == "sa4r2_dynamic_run02"


def test_rebuild_removes_stale_links(tmp_path):
    """★ 不整棵刪掉重建的話，改過名的舊連結會留在樹裡指向不存在的檔案，
    而使用者看不出哪個才是現在的。"""
    root = tmp_path / "rec"
    root.mkdir()
    d = _mk_run(root, "sa4r2_mixed_run01", "mixed", 1)
    build(root, {})
    stale = (root / BROWSE_DIRNAME / MAIN_DIRNAME / "模型sa4r2"
             / "混合_ORCA互動_第1趟_靜6動8_俯視.mp4")
    assert stale.exists()
    (d / "run.json").write_text(json.dumps(
        {"tag": "sa4r2_mixed_run01", "scenario": "static", "run_index": 1,
         "crowd_mode": "orca"}))
    build(root, {})
    assert not stale.is_symlink()


def test_skips_archive_and_browse_dirs(tmp_path):
    """★ 作廢存檔（`_` 開頭）與索引樹本身（`0` 開頭）不可被收進索引。"""
    root = tmp_path / "rec"
    root.mkdir()
    _mk_run(root, "sa4r2_mixed_run03", "mixed", 3)
    _mk_run(root, "_作廢_v1", "mixed", 3)
    build(root, {})
    names = [p.name for p in (root / BROWSE_DIRNAME / RAW_DIRNAME).iterdir()]
    assert len(names) == 1
    build(root, {})              # 第二次不能把 00_影片總覽 當成一趟收進去
    assert len([p.name for p in (root / BROWSE_DIRNAME / RAW_DIRNAME).iterdir()]) == 1


def test_missing_arm_root_is_skipped_not_fatal(tmp_path):
    """★ 對照組還沒跑時不該整支失敗。"""
    root = tmp_path / "rec"
    root.mkdir()
    _mk_run(root, "sa4r2_mixed_run04", "mixed", 4)
    stats = build(root, {"speed_0p6": tmp_path / "nope"})
    assert MAIN_DIRNAME in stats


def test_counts_come_from_the_run_log_not_the_plan_table(tmp_path):
    """★★ 數量要從該趟自己的 log 數 —— 執行期會停用擠在一起或太靠近車的角色，
    查 `scene_variants` 的計畫表會讓檔名寫出場上其實沒有的數量。"""
    root = tmp_path / "rec"
    root.mkdir()
    # 計畫表的 run4 是障礙 6 / 走動 8；這趟 log 說只剩 走動 6（兩人被停用）
    _mk_run(root, "sa4r2_mixed_run04", "mixed", 4,
            obstacles=6, chars=11, walking=6, standing=5)
    build(root, {})
    names = [p.name for p in
             (root / BROWSE_DIRNAME / MAIN_DIRNAME / "模型sa4r2").iterdir()]
    assert all("靜6動6" in n for n in names), names


def test_missing_log_is_labelled_unknown_not_zero(tmp_path):
    """★ 讀不到 log 就寫「數量不明」，不可填 0 假裝場上是空的。"""
    root = tmp_path / "rec"
    root.mkdir()
    d = _mk_run(root, "sa4r2_mixed_run04", "mixed", 4)
    (d / "isaac_nav.log").unlink()
    build(root, {})
    names = [p.name for p in
             (root / BROWSE_DIRNAME / MAIN_DIRNAME / "模型sa4r2").iterdir()]
    assert all("數量不明" in n for n in names), names
