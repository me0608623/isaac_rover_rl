"""中文索引命名的測試。"""

from __future__ import annotations

import pytest

from browse_names import (ARM_DIR, ARM_ZH, CAMERA_ZH, CROWD_MODE_ZH,
                          SCENARIO_ZH, counts_label, crowd_mode_label,
                          run_label, video_name)


def test_every_scenario_and_camera_has_a_chinese_name():
    """★ 少一個對照就會產出半英文的檔名，或直接 ValueError 中斷整棵樹。"""
    from scenarios import SCENARIO_NAMES
    from sim_cameras import CAMERAS

    assert set(SCENARIO_ZH) == set(SCENARIO_NAMES)
    assert set(CAMERA_ZH) == {c.name for c in CAMERAS}


def test_static_run_is_not_labelled_orca():
    """★★ `static` 的 run.json 照樣記 `crowd_mode: orca`（那是 CLI 參數），
    但 `walks_enabled=False`、沒有人在走。只看 crowd_mode 會把純靜態的片子
    標成「ORCA互動」—— 檔名自己說謊。"""
    assert crowd_mode_label("orca", n_dynamic=0) == "不走動"
    assert video_name("static", 4, "topdown", (14, 0), "orca") == \
        "純靜態_不走動_第4趟_靜14動0_俯視.mp4"


def test_orca_and_path_are_distinguishable_in_the_name():
    """★ ORCA 互動避讓 vs 走固定路線是對照組的**唯一**差別，
    檔名看不出來就分不出哪個是哪個。"""
    a = video_name("mixed", 4, "topdown", (6, 8), "orca")
    b = video_name("mixed", 4, "topdown", (6, 8), "path")
    assert "ORCA互動" in a and "固定路線" in b
    assert a != b


def test_counts_label():
    assert counts_label((6, 8)) == "靜6動8"
    assert counts_label((14, 0)) == "靜14動0"


def test_unknown_counts_are_not_faked_as_zero():
    """★ log 讀不出來時填 0 會讓假數字看起來一樣可信。"""
    assert counts_label(None) == "數量不明"
    assert "數量不明" in run_label("mixed", 1, None, "orca")


def test_run_label_examples():
    assert run_label("mixed", 4, (6, 8), "orca") == "混合_ORCA互動_第4趟_靜6動8"
    assert run_label("dynamic", 4, (5, 8), "orca") == "純動態_ORCA互動_第4趟_靜5動8"


def test_dynamic_scenario_still_reports_its_static_bodies():
    """★★ `dynamic` 印「靜態障礙 0/18 啟用」只關圓柱，站立人物仍在場。
    名字寫「靜0」會讓人以為那是完全沒有靜態物的場景。"""
    assert "靜5" in run_label("dynamic", 4, (5, 8), "orca")


def test_unknown_crowd_mode_shows_through_instead_of_vanishing():
    """★ 新增走法時不該安靜地變成空字串。"""
    assert "newmode" in crowd_mode_label("newmode", n_dynamic=3)


def test_unknown_scenario_or_camera_raises():
    with pytest.raises(ValueError):
        run_label("nope", 1, (1, 1), "orca")
    with pytest.raises(ValueError):
        video_name("static", 1, "nope", (1, 1), "orca")


def test_arm_names_cover_the_three_ablations_and_flag_non_orca():
    assert set(ARM_DIR) == set(ARM_ZH) == {"crowd_path", "speed_0p6", "speed_1p0"}
    assert "非ORCA" in ARM_DIR["crowd_path"]
    assert set(CROWD_MODE_ZH) == {"orca", "path"}


def test_arm_dir_names_sort_in_design_order():
    """★ 編號前綴讓三組對照照設計順序排，而不是按注音/筆畫亂排。"""
    assert sorted(ARM_DIR.values()) == [
        ARM_DIR["crowd_path"], ARM_DIR["speed_0p6"], ARM_DIR["speed_1p0"]]
