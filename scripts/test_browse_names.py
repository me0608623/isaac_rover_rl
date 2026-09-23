"""中文索引命名的測試。"""

from __future__ import annotations

import pytest

from browse_names import (ARM_ZH, CAMERA_ZH, SCENARIO_ZH, run_label,
                          scene_load, video_name)


def test_every_scenario_and_camera_has_a_chinese_name():
    """★ 少一個對照就會產出半英文的檔名，或直接 KeyError 中斷整棵樹。"""
    from scenarios import SCENARIO_NAMES
    from sim_cameras import CAMERAS

    assert set(SCENARIO_ZH) == set(SCENARIO_NAMES)
    assert set(CAMERA_ZH) == {c.name for c in CAMERAS}


def test_scene_load_does_not_claim_obstacles_in_dynamic():
    """★ dynamic 情境沒有靜態障礙，名字裡不能出現「障礙」——
    否則影片名本身就在說謊。"""
    assert "障礙" not in scene_load("dynamic", 6, 8)
    assert scene_load("dynamic", 6, 8) == "行人8"


def test_scene_load_does_not_claim_walkers_in_static():
    """★ static 情境的行人不走動，名字裡不該出現行人數。"""
    assert "行人" not in scene_load("static", 6, 8)
    assert scene_load("static", 6, 8) == "障礙6"


def test_mixed_names_both():
    assert scene_load("mixed", 6, 8) == "障礙6行人8"


def test_run_label():
    assert run_label("mixed", 4, 6, 8) == "靜動態混合_第4趟_障礙6行人8"


def test_video_name():
    assert video_name("static", 1, "topdown", 3, 2) == "靜態障礙_第1趟_障礙3_俯視.mp4"


def test_unknown_scenario_or_camera_raises():
    """★ 靜靜回一個怪名字的話，會生出一堆對不到來源的連結才被發現。"""
    with pytest.raises(ValueError):
        run_label("nope", 1, 1, 1)
    with pytest.raises(ValueError):
        video_name("static", 1, "nope", 1, 1)


def test_arm_labels_cover_the_three_ablations():
    assert set(ARM_ZH) == {"crowd_path", "speed_0p6", "speed_1p0"}
