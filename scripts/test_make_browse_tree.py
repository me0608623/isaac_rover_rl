"""中文索引樹的測試。"""

from __future__ import annotations

import json
import os

from make_browse_tree import BROWSE_DIRNAME, MAIN_DIRNAME, RAW_DIRNAME, build


def _mk_run(root, tag, scenario, idx, cams=("topdown", "chase", "oblique")):
    d = root / tag
    (d / "video").mkdir(parents=True)
    (d / "run.json").write_text(json.dumps(
        {"tag": tag, "scenario": scenario, "run_index": idx}))
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
    assert got == ["靜動態混合_第4趟_障礙6行人8_俯視.mp4",
                   "靜動態混合_第4趟_障礙6行人8_斜前方.mp4",
                   "靜動態混合_第4趟_障礙6行人8_車後.mp4"]


def test_links_are_relative_and_resolve(tmp_path):
    """★ 絕對路徑的連結在 recordings/ 被搬走或掛到別台之後全部斷掉。"""
    root = tmp_path / "rec"
    root.mkdir()
    _mk_run(root, "sa4r2_static_run01", "static", 1)
    build(root, {})
    link = (root / BROWSE_DIRNAME / MAIN_DIRNAME / "模型sa4r2"
            / "靜態障礙_第1趟_障礙3_俯視.mp4")
    assert link.is_symlink()
    assert not os.path.isabs(os.readlink(link))
    assert link.resolve().read_text() == "x"


def test_raw_data_link_points_at_the_run_directory(tmp_path):
    root = tmp_path / "rec"
    root.mkdir()
    _mk_run(root, "sa4r2_dynamic_run02", "dynamic", 2)
    build(root, {})
    link = (root / BROWSE_DIRNAME / RAW_DIRNAME
            / "模型sa4r2_動態行人_第2趟_行人4")
    assert link.resolve().name == "sa4r2_dynamic_run02"


def test_rebuild_removes_stale_links(tmp_path):
    """★ 不整棵刪掉重建的話，改過名的舊連結會留在樹裡指向不存在的檔案，
    而使用者看不出哪個才是現在的。"""
    root = tmp_path / "rec"
    root.mkdir()
    d = _mk_run(root, "sa4r2_mixed_run01", "mixed", 1)
    build(root, {})
    stale = (root / BROWSE_DIRNAME / MAIN_DIRNAME / "模型sa4r2"
             / "靜動態混合_第1趟_障礙3行人2_俯視.mp4")
    assert stale.exists()
    (d / "run.json").write_text(json.dumps(
        {"tag": "sa4r2_mixed_run01", "scenario": "static", "run_index": 1}))
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
