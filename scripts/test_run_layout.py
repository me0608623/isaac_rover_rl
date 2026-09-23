"""`recordings/` 版面的測試。"""

from __future__ import annotations

import json

import pytest

from run_layout import (BROWSE_DIRNAME, model_dir_name, model_from_dir_name,
                        run_dir_for, run_dirs)


def _mk(d, tag="t"):
    d.mkdir(parents=True, exist_ok=True)
    (d / "run.json").write_text(json.dumps({"tag": tag}))
    return d


def test_model_dir_name_roundtrip():
    assert model_dir_name("sa4r2") == "模型sa4r2"
    assert model_from_dir_name("模型sa4r2") == "sa4r2"
    assert model_from_dir_name("sa4r2_static_run01") is None
    assert model_from_dir_name("模型") is None


def test_empty_model_raises():
    """★ 空字串會產生資料夾名「模型」，整批倒進同一個夾子。"""
    with pytest.raises(ValueError):
        model_dir_name("")


def test_run_dir_for():
    assert str(run_dir_for("rec", "sa4r2", "sa4r2_static_run01")) == \
        "rec/模型sa4r2/sa4r2_static_run01"


def test_finds_runs_under_model_dirs(tmp_path):
    """★ 這是分類之後的正常版面 —— 找不到就等於所有分析輸出空表。"""
    _mk(tmp_path / "模型sa4r2" / "sa4r2_static_run01")
    _mk(tmp_path / "模型sa5r2" / "sa5r2_mixed_run04")
    got = [p.name for p in run_dirs(tmp_path)]
    assert got == ["sa4r2_static_run01", "sa5r2_mixed_run04"]


def test_still_finds_runs_directly_under_root(tmp_path):
    """★ 對照組（recordings_abl/<arm>/<tag>）沒有模型層，不可因此漏掉。"""
    _mk(tmp_path / "sa4r2_static_run01")
    assert [p.name for p in run_dirs(tmp_path)] == ["sa4r2_static_run01"]


def test_mixed_layout_is_not_double_counted(tmp_path):
    _mk(tmp_path / "sa4r2_static_run01")
    _mk(tmp_path / "模型sa4r3" / "sa4r3_mixed_run02")
    assert len(run_dirs(tmp_path)) == 2


def test_browse_tree_symlinks_are_not_counted_twice(tmp_path):
    """★★ 索引樹裡是指向各趟的符號連結；跟著收會讓每趟被算兩次，
    而彙總表看起來只是「趟數變兩倍」，很難看出是重複。"""
    real = _mk(tmp_path / "模型sa4r2" / "sa4r2_static_run01")
    link_dir = tmp_path / BROWSE_DIRNAME / "05_每趟原始資料_bag與CSV"
    link_dir.mkdir(parents=True)
    (link_dir / "模型sa4r2_靜態障礙_第1趟").symlink_to(real)
    assert [p.name for p in run_dirs(tmp_path)] == ["sa4r2_static_run01"]


def test_archive_dirs_are_skipped(tmp_path):
    _mk(tmp_path / "_作廢批次" / "old_run01")
    _mk(tmp_path / "模型sa4r2" / "sa4r2_static_run01")
    assert [p.name for p in run_dirs(tmp_path)] == ["sa4r2_static_run01"]


def test_missing_root_returns_empty_not_raises(tmp_path):
    assert run_dirs(tmp_path / "nope") == []


#: 會去 recordings/ 找「一趟」的所有程式。
_DISCOVERY_TOOLS = ("check_recordings", "make_readme", "compare_arms",
                    "make_sync", "localization_error", "nearest_source",
                    "make_browse_tree")


@pytest.mark.parametrize("name", _DISCOVERY_TOOLS)
def test_every_tool_discovers_runs_through_run_layout(name):
    """★★ 每支都必須用 `run_layout.run_dirs`，不可自己 `root.iterdir()`。

    2026-09-23 把 `iterdir()` 換成 `run_dirs()` 時漏了 5 支的 import —— 而
    **整套測試照樣全過**，因為測試只碰純函式、不碰 `main()`。那五支要等真的
    去跑才會 NameError；更糟的情況是它們各自又長回自己的搜尋邏輯，
    於是同一批資料在不同報表裡趟數不一樣。
    """
    import importlib
    import inspect

    mod = importlib.import_module(name)
    assert hasattr(mod, "run_dirs"), f"{name} 沒有 import run_layout.run_dirs"
    src = inspect.getsource(mod)
    assert "root.iterdir()" not in src, (
        f"{name} 還在自己列目錄 —— 「什麼算一趟」只能有一處定義")
