"""中文索引樹的測試。"""

from __future__ import annotations

import json
import os

from make_browse_tree import BROWSE_DIRNAME, MAIN_DIRNAME, RAW_DIRNAME, build


def _mp4s(d):
    """索引樹分層之後（模型/路線/情境），影片不在第一層 —— 遞迴收。"""
    return sorted(p.name for p in d.rglob("*.mp4"))


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
    got = _mp4s(root / BROWSE_DIRNAME / MAIN_DIRNAME / "模型sa4r2")
    assert got == ["sa4r2_混合_ORCA互動_第4趟_靜6動8_俯視.mp4",
                   "sa4r2_混合_ORCA互動_第4趟_靜6動8_斜前方.mp4",
                   "sa4r2_混合_ORCA互動_第4趟_靜6動8_車後.mp4"]


def test_directory_links_are_relative_and_resolve(tmp_path):
    """★ 資料夾只能用符號連結（hard link 不能指資料夾），而且要用相對路徑 ——
    絕對路徑在 recordings/ 被搬走或掛到別台之後全部斷掉。

    影片不走這條路：影片是 hard link，見
    `test_video_entries_are_real_files_not_shortcuts`。
    """
    root = tmp_path / "rec"
    root.mkdir()
    _mk_run(root, "sa4r2_static_run01", "static", 1, obstacles=3,
            chars=5, walking=0, standing=3)
    build(root, {})
    link = (root / BROWSE_DIRNAME / RAW_DIRNAME
            / "模型sa4r2_純靜態_不走動_第1趟_靜5動0")
    assert link.is_symlink()
    assert not os.path.isabs(os.readlink(link))
    assert link.resolve().name == "sa4r2_static_run01"


def test_raw_data_link_points_at_the_run_directory(tmp_path):
    root = tmp_path / "rec"
    root.mkdir()
    _mk_run(root, "sa4r2_dynamic_run02", "dynamic", 2, obstacles=0,
            chars=8, walking=4, standing=4)
    build(root, {})
    link = (root / BROWSE_DIRNAME / RAW_DIRNAME
            / "模型sa4r2_動態_ORCA互動_第2趟_靜4動4")
    assert link.resolve().name == "sa4r2_dynamic_run02"


def test_rebuild_removes_stale_links(tmp_path):
    """★ 不整棵刪掉重建的話，改過名的舊連結會留在樹裡指向不存在的檔案，
    而使用者看不出哪個才是現在的。"""
    root = tmp_path / "rec"
    root.mkdir()
    d = _mk_run(root, "sa4r2_mixed_run01", "mixed", 1)
    build(root, {})
    stale = (root / BROWSE_DIRNAME / MAIN_DIRNAME / "模型sa4r2" / "路線未標示_舊錄影"
            / "3_混合_靜止障礙加走動行人"
             / "sa4r2_混合_ORCA互動_第1趟_靜6動8_俯視.mp4")
    assert stale.exists()
    (d / "run.json").write_text(json.dumps(
        {"tag": "sa4r2_mixed_run01", "scenario": "static", "run_index": 1,
         "crowd_mode": "orca"}))
    build(root, {})
    assert not stale.exists()


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
    names = _mp4s(root / BROWSE_DIRNAME / MAIN_DIRNAME / "模型sa4r2")
    assert all("靜6動6" in n for n in names), names


def test_missing_log_is_labelled_unknown_not_zero(tmp_path):
    """★ 讀不到 log 就寫「數量不明」，不可填 0 假裝場上是空的。"""
    root = tmp_path / "rec"
    root.mkdir()
    d = _mk_run(root, "sa4r2_mixed_run04", "mixed", 4)
    (d / "isaac_nav.log").unlink()
    build(root, {})
    names = _mp4s(root / BROWSE_DIRNAME / MAIN_DIRNAME / "模型sa4r2")
    assert all("數量不明" in n for n in names), names


def test_each_model_folder_gets_its_own_chinese_index(tmp_path):
    """★ 使用者會直接點進 `recordings/模型sa4r2/`，而那底下是英文 tag
    資料夾。原地要有一份中文索引，不然走進去就看不懂。"""
    from make_browse_tree import MODEL_INDEX_DIRNAME

    root = tmp_path / "rec"
    (root / "模型sa4r2").mkdir(parents=True)
    _mk_run(root / "模型sa4r2", "sa4r2_mixed_run04", "mixed", 4)
    build(root, {})
    got = _mp4s(root / "模型sa4r2" / MODEL_INDEX_DIRNAME)
    assert got == ["sa4r2_混合_ORCA互動_第4趟_靜6動8_俯視.mp4",
                   "sa4r2_混合_ORCA互動_第4趟_靜6動8_斜前方.mp4",
                   "sa4r2_混合_ORCA互動_第4趟_靜6動8_車後.mp4"]


def test_model_index_is_not_mistaken_for_a_run(tmp_path):
    """★★ 原地索引就放在模型資料夾裡，`run_dirs` 不能把它當成一趟，
    也不能跟著裡面的連結把同一趟算兩次。"""
    from run_layout import run_dirs

    root = tmp_path / "rec"
    (root / "模型sa4r2").mkdir(parents=True)
    _mk_run(root / "模型sa4r2", "sa4r2_mixed_run04", "mixed", 4)
    build(root, {})
    assert [p.name for p in run_dirs(root)] == ["sa4r2_mixed_run04"]


def test_model_index_is_rebuilt_not_appended(tmp_path):
    """★ 改名後舊連結要消失，否則同一趟在原地索引裡出現兩個名字。"""
    from make_browse_tree import MODEL_INDEX_DIRNAME

    root = tmp_path / "rec"
    (root / "模型sa4r2").mkdir(parents=True)
    d = _mk_run(root / "模型sa4r2", "sa4r2_mixed_run04", "mixed", 4)
    build(root, {})
    idx = root / "模型sa4r2" / MODEL_INDEX_DIRNAME
    assert len(_mp4s(idx)) == 3
    (d / "isaac_nav.log").write_text(
        "[run_isaac_sim] 情境 mixed／變體 run4：靜態障礙 3/18 啟用　行人走動 開\n"
        "[run_isaac_sim] 程序化步態：5 人 / 40 個擺動關節 / 2 人沿路徑移動"
        "　站立人物擺位 3 個　腳底對地 3 個\n")
    build(root, {})
    got = _mp4s(idx)
    assert len(got) == 3 and all("靜3動2" in g for g in got), got


def test_video_entries_are_real_files_not_shortcuts(tmp_path):
    """★★ 影片必須是 hard link（在檔案總管裡就是普通檔案）。

    2026-09-23 使用者把索引裡的影片複製到 `00_影片總覽/上傳/` 之後打不開：
    複製出來的還是捷徑，而捷徑寫的是相對路徑，換了層數就指到不存在的地方。
    """
    root = tmp_path / "rec"
    root.mkdir()
    _mk_run(root, "sa4r2_mixed_run04", "mixed", 4)
    build(root, {})
    link = (root / BROWSE_DIRNAME / MAIN_DIRNAME / "模型sa4r2" / "路線未標示_舊錄影"
            / "3_混合_靜止障礙加走動行人"
            / "sa4r2_混合_ORCA互動_第4趟_靜6動8_俯視.mp4")
    assert not link.is_symlink(), "影片不可以是符號連結"
    assert link.is_file() and link.read_text() == "x"
    assert link.stat().st_nlink >= 2, "應與本體共用同一個 inode"


def test_copying_a_video_elsewhere_still_opens(tmp_path):
    """★★ 直接重現使用者的操作：複製到另一個深度不同的資料夾後要還能開。"""
    import shutil as _sh

    root = tmp_path / "rec"
    root.mkdir()
    _mk_run(root, "sa4r2_mixed_run04", "mixed", 4)
    build(root, {})
    src = (root / BROWSE_DIRNAME / MAIN_DIRNAME / "模型sa4r2" / "路線未標示_舊錄影"
            / "3_混合_靜止障礙加走動行人"
           / "sa4r2_混合_ORCA互動_第4趟_靜6動8_俯視.mp4")
    dest_dir = root / BROWSE_DIRNAME / "上傳"
    dest_dir.mkdir()
    _sh.copy(src, dest_dir / src.name)       # 檔案總管的「複製」
    assert (dest_dir / src.name).read_text() == "x"


def test_rebuild_keeps_user_folders(tmp_path):
    """★★ 使用者會在 `00_影片總覽/` 裡自己開資料夾（例如 `上傳/`）放要給人的
    片子。原本的 rmtree(browse) 下次重建就把它整個刪掉了。"""
    root = tmp_path / "rec"
    root.mkdir()
    _mk_run(root, "sa4r2_mixed_run04", "mixed", 4)
    build(root, {})
    mine = root / BROWSE_DIRNAME / "上傳"
    mine.mkdir()
    (mine / "給老師.mp4").write_text("keep me")
    build(root, {})
    assert (mine / "給老師.mp4").read_text() == "keep me"


def test_rebuild_still_removes_its_own_stale_blocks(tmp_path):
    """★ 但自己產生的區塊（NN_ 開頭）改名後要清掉，不可兩個名字並存。"""
    root = tmp_path / "rec"
    root.mkdir()
    _mk_run(root, "sa4r2_mixed_run04", "mixed", 4)
    build(root, {})
    stale = root / BROWSE_DIRNAME / "02_對照_舊名字"
    stale.mkdir()
    (stale / "x.mp4").write_text("old")
    build(root, {})
    assert not stale.exists()


def test_videos_are_grouped_by_model_route_scenario(tmp_path):
    """★ 2026-09-24 使用者：「把檔案命名清楚、整理歸類」。一個模型 72 支
    影片平鋪在一層很難找 —— 分成 模型/路線/情境 三層，檔名開頭帶模型名
    （複製到別處後仍認得出來）。"""
    root = tmp_path / "rec"
    (root / "模型sa4r2").mkdir(parents=True)
    d = _mk_run(root / "模型sa4r2", "sa4r2_c36_static_run02", "static", 2,
                obstacles=4, chars=5, walking=0, standing=2)
    meta = json.loads((d / "run.json").read_text())
    meta["route_key"] = "c36"
    (d / "run.json").write_text(json.dumps(meta))
    build(root, {})
    leaf = (root / BROWSE_DIRNAME / MAIN_DIRNAME / "模型sa4r2"
            / "路線B_c28往返c36_延伸到c36" / "1_純靜態_只有靜止障礙")
    assert _mp4s(leaf) == [
        "sa4r2_路線c36_純靜態_不走動_第2趟_靜7動0_俯視.mp4",
        "sa4r2_路線c36_純靜態_不走動_第2趟_靜7動0_斜前方.mp4",
        "sa4r2_路線c36_純靜態_不走動_第2趟_靜7動0_車後.mp4"]
    # 原地索引同樣分層（少了模型那一層，因為已經在模型資料夾裡）
    from make_browse_tree import MODEL_INDEX_DIRNAME
    assert _mp4s(root / "模型sa4r2" / MODEL_INDEX_DIRNAME
                 / "路線B_c28往返c36_延伸到c36" / "1_純靜態_只有靜止障礙") == _mp4s(leaf)
