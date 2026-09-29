"""壓力測試行駛時間／卡住指標測試。"""

from __future__ import annotations

import json

from driving_time import collect, main, nav_samples, stuck_intervals


def test_stuck_interval_requires_five_seconds_inside_ten_centimetres():
    samples = [(i * 0.1, 0.005 * (i % 3), 0.0) for i in range(61)]
    assert stuck_intervals(samples) == [(0.0, 6.0)]


def test_slow_but_continuous_motion_is_not_stuck():
    samples = [(i * 0.1, i * 0.01, 0.0) for i in range(101)]
    assert stuck_intervals(samples) == []


def test_separate_stationary_periods_are_counted_separately():
    first = [(i * 0.1, 0.0, 0.0) for i in range(61)]
    move = [(6.1, 1.0, 0.0)]
    second = [(6.2 + i * 0.1, 1.0, 0.0) for i in range(61)]
    got = stuck_intervals(first + move + second)
    assert len(got) == 2
    assert got[0] == (0.0, 6.0)


def test_nav_samples_skips_bad_and_nonfinite_rows(tmp_path):
    p = tmp_path / "nav.csv"
    p.write_text("t,map_x,map_y\n0.0,1.0,2.0\nbad,1,2\n1.0,nan,2.0\n")
    assert nav_samples(p) == [(0.0, 1.0, 2.0)]


def test_stress_report_counts_timeout_stuck_and_truth_clearance(tmp_path, capsys):
    root = tmp_path / "recordings_stress"
    run = root / "模型sa4r2" / "sa4r2_c27_mixed_s1_run01"
    (run / "nav").mkdir(parents=True)
    (run / "run.json").write_text(json.dumps({
        "tag": run.name, "model": "sa4r2", "route_key": "c27",
        "density": "S1", "scenario": "mixed", "run_index": 1,
        "leg_timeout_s": 300,
        "collisions": {"clearance": {"complete": True, "min_m": 0.18,
                                      "method": "physx"}},
    }))
    (run / "nav.log").write_text("c28→c27 FAIL 300.0s 1.00m\n")
    nav = run / "nav" / f"{run.name}_leg1_c28_to_c27.csv"
    nav.write_text("t,map_x,map_y\n" + "".join(
        f"{i/10:.1f},0.0,0.0\n" for i in range(61)))
    rows = collect(root)
    assert rows[0]["timeout"] is True
    assert rows[0]["stuck_segments"] == 1
    assert rows[0]["truth_clearance_m"] == 0.18

    out_csv = root / "metrics.csv"
    assert main([str(root), "--csv", str(out_csv)]) == 0
    report = capsys.readouterr().out
    assert "0/1 | 0.0%" in report
    assert "| 1 | 1 |" in report       # 逾時 1、卡住 1
    assert out_csv.exists()


def test_formal_report_keeps_legacy_columns_and_table(tmp_path, capsys):
    root = tmp_path / "recordings"
    run = root / "模型sa4r2" / "sa4r2_c27_static_run01"
    (run / "nav").mkdir(parents=True)
    (run / "run.json").write_text(json.dumps({
        "tag": run.name, "model": "sa4r2", "route_key": "c27",
        "scenario": "static", "run_index": 1,
    }))
    (run / "nav.log").write_text(
        "c28→c27 OK 10.0s 5.00m\n"
        "c27→c28 OK 11.0s 5.10m\n")
    for i, (a, b, end) in enumerate((("c28", "c27", 10), ("c27", "c28", 11)), 1):
        (run / "nav" / f"{run.name}_leg{i}_{a}_to_{b}.csv").write_text(
            "t,map_x,map_y\n0,0,0\n" + f"{end},5,0\n")
    out_csv = tmp_path / "formal.csv"
    assert main([str(root), "--csv", str(out_csv)]) == 0
    report = capsys.readouterr().out
    assert "去：行駛 s" in report
    assert "成功率" not in report
    assert out_csv.read_text().splitlines()[0] == \
        "model,route,scenario,run,leg,seg,ok,total,dist,drive,wait"
