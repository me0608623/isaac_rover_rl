"""錄影情境設定的測試。"""

from __future__ import annotations

import pytest

from scenarios import SCENARIO_NAMES, scenario_config


def test_three_scenarios_are_defined():
    assert SCENARIO_NAMES == ("static", "dynamic", "mixed")


def test_static_has_obstacles_but_nobody_walks():
    c = scenario_config("static")
    assert c.obstacles_enabled is True
    assert c.walks_enabled is False


def test_dynamic_has_walkers_but_no_static_obstacles():
    c = scenario_config("dynamic")
    assert c.obstacles_enabled is False
    assert c.walks_enabled is True


def test_mixed_has_both():
    c = scenario_config("mixed")
    assert c.obstacles_enabled is True
    assert c.walks_enabled is True


def test_every_scenario_differs_from_the_others():
    """★ 三個情境若有兩個設定相同，錄出來的對照組就沒有意義。"""
    seen = {(scenario_config(n).obstacles_enabled, scenario_config(n).walks_enabled)
            for n in SCENARIO_NAMES}
    assert len(seen) == len(SCENARIO_NAMES)


def test_unknown_scenario_is_rejected_loudly():
    """★ 打錯情境名不能靜靜地當成預設值 —— 那會錄出 12 段一模一樣的影片。"""
    with pytest.raises(ValueError, match="mixed"):
        scenario_config("dyanmic")


def test_config_is_immutable():
    c = scenario_config("mixed")
    with pytest.raises(Exception):
        c.walks_enabled = False
