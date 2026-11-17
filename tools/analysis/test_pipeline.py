#!/usr/bin/env python3
"""Tests of the slip analysis pipeline (plain asserts, no pytest):  python3 tools/analysis/test_pipeline.py

  parsing edge cases, episode labelling (late marks, missing end, next mark, outside MONITOR), scoring on a hand-built
  case, the Python stand-in on hand-built signals, the subprocess replay path, the C++ replay tool against the
  stand-in (if built), and an end-to-end synthetic run (detection >= 80 %, false alarms <= 0.5/min).
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
import time

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402

import make_synthetic  # noqa: E402
import replay_py  # noqa: E402
import sgdata  # noqa: E402
import slipeval as se  # noqa: E402
import tune_slip  # noqa: E402

TMP = tempfile.mkdtemp(prefix="sg_test_")
SYN = os.path.join(TMP, "syn")


def row(seq, t, state, f=(1.0, 1.0, 0, 0, 0, 0), event="", obj=True, rng=80.0):
    fs = ["" if v is None else f"{v:.3f}" for v in f]
    m = [f"{50.0:.1f}", f"{rng:.1f}", f"{30.0:.1f}", "20.0", "", "21.0"] if obj else [""] * 6
    return ",".join([str(seq), str(t), state, "-", "1", "1500", "0", "0", "0.0", *fs, *fs, "1", "1", "0",
                     str(seq // 7), "1" if obj else "0", *m, "medium", "sphere" if obj else "", event])


def write_rec(folder, name, rows, comments=("# SenseGrip recording x", "# firmware: test"), tof_lines=None):
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, name), "w") as fh:
        fh.write("\n".join(list(comments) + [",".join(sgdata.MAIN_HEADER)] + rows) + "\n")
    if tof_lines is not None:
        with open(os.path.join(folder, name[:-4] + "_tof.csv"), "w") as fh:
            fh.write("\n".join(["# frames", ",".join(sgdata.TOF_HEADER)] + tof_lines) + "\n")


def monitor_timeline(n_pre=10, n_mon=600, marks=None, drop_at=None, drop_to=0.3):
    """GRASP rows then MONITOR rows at 10 ms; marks: {row index: event}; force drop at row drop_at."""
    rows, marks = [], marks or {}
    for i in range(n_pre + n_mon):
        st = "GRASP" if i < n_pre else "MONITOR"
        f1 = drop_to if (drop_at is not None and i >= drop_at) else 1.0
        ev = marks.get(i, "state:MONITOR" if i == n_pre else "")
        rows.append(row(1000 + i, 10_000 + 10 * i, st, (f1, 1.0, 0, 0, 0, 0), ev))
    return rows


# ---------------------------------------------------------------------------------------------------------------
def test_parsing():
    d = os.path.join(TMP, "parse")
    rows = monitor_timeline(n_mon=100)
    rows[30] = row(1030, 10_300, "MONITOR", (None, 1.0, 0, 0, 0, 0))          # empty force field
    del rows[50:55]                                                             # gap of 5 rows
    rows.append("1999,99999,MONITOR,-,1")                                       # truncated last line
    write_rec(d, "grasp_7_medium_20261118-101502.csv", rows,
              comments=("# SenseGrip recording grasp_7", "# firmware: phase4-rec",
                        "# started: 2026-11-18T10:15:02.000Z | rows: 105 | result: IDLE | GAP: rows missing"))
    write_rec(d, "manual_8_20261118-101900.csv", monitor_timeline(n_pre=0, n_mon=50, marks={10: "mark:pull"}),
              tof_lines=[f"0,10005,1001,1500,7," + ",".join(["100"] * 192)])
    with open(os.path.join(d, "notes.csv"), "w") as fh:
        fh.write("file,object,trial,notes\nmanual_8_20261118-101900.csv,Mug,C pull,manual, jogged\n")
    recs = sgdata.load_dataset(d)
    assert [r.name for r in recs] == ["grasp_7_medium_20261118-101502.csv", "manual_8_20261118-101900.csv"]
    g, m = recs
    assert len(g.rows) == 105, len(g.rows)                       # garbage line dropped
    assert g.gap_rows == 5 and g.gap_flag
    assert g.tof is None and g.tof_path is None
    assert np.isnan(g.rows["f1"].iloc[30]) and g.rows["event"].iloc[0] == ""
    assert g.object == "medium" and g.trial == "A quiet" and g.cls == "medium" and g.kind == "grasp"
    assert np.isnan(g.rows["rad2"].iloc[0])
    assert m.kind == "manual" and m.cls == "" and m.object == "mug" and m.notes == "manual,jogged"
    assert m.tof is not None and m.arrays()["frame_new"][1] and m.arrays()["last_frame"][0] == -1
    assert g.arrays()["frame_new"].sum() == 0 and g.arrays()["frames"].shape == (0, 3, 64)
    # the stand-in runs on it (gap, NaN force, no frames) and prints one line per row
    out = replay_py.format_csv(g.arrays(), replay_py.run(g.arrays()))
    assert out.count("\n") == 106 and out.startswith(replay_py.OUT_HEADER)
    print("ok parsing")


def test_episodes():
    d = os.path.join(TMP, "eps")
    # MONITOR from row 10; pull at row 300 (late mark), end at 400; other at 500 w/o end -> +2 s cut by
    # recording end; twist at 200 cut by... no: order matters -> use separate file
    write_rec(d, "grasp_1_small_20261118-100000.csv",
              monitor_timeline(n_mon=900, marks={150: "mark:pull", 250: "mark:end", 400: "mark:other",
                                                 450: "mark:twist", 700: "mark:slide"}))
    write_rec(d, "grasp_2_small_20261118-100100.csv",
              monitor_timeline(n_pre=100, n_mon=200, marks={50: "mark:pull", 120: "mark:end", 200: "mark:end"}))
    r1, r2 = sgdata.load_dataset(d)
    e = r1.episodes
    assert [(x.kind, x.type) for x in e] == [("slip", "pull"), ("disturbance", "other"), ("slip", "twist"),
                                              ("slip", "slide")]
    assert (e[0].t_mark, e[0].t_end, e[0].end_by) == (11_500, 12_500, "end")
    assert e[0].window() == (11_000, 13_000)
    assert (e[1].t_end, e[1].end_by) == (14_500, "next_mark")            # other cut by the twist mark
    assert (e[2].t_end, e[2].end_by) == (17_000, "next_mark")            # twist: +3 s would be 17_500
    assert (e[3].t_end, e[3].end_by) == (10_000 + 10 * 909, "recording_end")   # slide: +3 s past the end
    assert all(x.scorable for x in e)
    assert r1.trial == "mixed pull+slide+twist", r1.trial
    assert len(r2.episodes) == 1 and not r2.episodes[0].scorable and "GRASP" in r2.episodes[0].why_not
    print("ok episodes")


def test_scoring():
    d = os.path.join(TMP, "score")
    # 60 s of MONITOR; slip mark at 20 s (late mark: event 0.2 s before it), end 21 s; disturbance 40-41 s
    n_pre, n = 10, 6000
    write_rec(d, "grasp_1_medium_20261118-100000.csv",
              monitor_timeline(n_pre=n_pre, n_mon=n, marks={n_pre + 2000: "mark:pull", n_pre + 2100: "mark:end",
                                                            n_pre + 4000: "mark:other", n_pre + 4100: "mark:end",
                                                            n_pre + 5000: "mark:twist"}))
    rec = sgdata.load_dataset(d)[0]
    rec.episodes[2].scorable = False                                     # pretend the twist mark was unscorable
    t = rec.rows["t_ms"].to_numpy()
    active = (rec.rows["state"] == "MONITOR").to_numpy() & (np.arange(len(t)) >= n_pre + 50)
    slip = np.zeros(len(t), bool)
    for i in (n_pre + 1980, n_pre + 2050, n_pre + 4050, n_pre + 1000, n_pre + 3000, n_pre + 2140, n_pre + 5010):
        slip[i] = True                 # detection (-200 ms), 2nd event same window, disturbance, 2 quiet FA,
    sc = se.score(rec, {"t": t, "active": active, "slip": slip})         # 1 inside end+0.5 s, 1 unscorable
    assert sc.slips == [("pull", True, -200, 10_000 + 10 * (n_pre + 2000))], sc.slips
    assert sc.dists == [True]
    assert len(sc.fa_times) == 2 and sc.unscorable_events == 1 and sc.n_events == 7
    win = (2100 + 50 - 2000 + 50) + (4100 + 50 - 4000 + 50) + (5000 + 300 + 50 - 5000 + 50)   # rows in windows
    exp_quiet = (n - 50 - win) * 0.01
    assert abs(sc.quiet_s - exp_quiet) < 0.05, (sc.quiet_s, exp_quiet)
    a = se.aggregate([sc])
    assert a["det_rate"] == 1.0 and a["lat_mean_ms"] == -200 and a["dist_rate"] == 1.0
    assert abs(a["fa_per_min"] - 2 / (exp_quiet / 60)) < 0.01
    assert not se.feasible(a, 0.5, 0.3)
    assert se.rank_key(a, 0.5, 0.3)[0] == 1
    print("ok scoring")


def test_standin_signals():
    d = os.path.join(TMP, "sig")
    # MONITOR from row 10 (t=10100); settle 500 ms -> armed at row 60; pad 1 drops 1.0 -> 0.3 at row 200
    write_rec(d, "grasp_1_medium_20261118-100000.csv", monitor_timeline(n_mon=600, drop_at=200))
    rec = sgdata.load_dataset(d)[0]
    o = replay_py.run(rec.arrays(), {})
    assert not o["active"][59] and o["active"][60]
    assert abs(o["fsr_drop"][200] - 0.7) < 1e-5
    assert not o["fsr_flag"][202] and o["fsr_flag"][203]                  # 30 ms confirm = 4th row
    assert np.nonzero(o["slip"])[0].tolist() == [203] and o["reason"][203] == 1
    assert not o["fsr_flag"][200 + 31]                                    # drop leaves the 300 ms window
    o = replay_py.run(rec.arrays(), {"mode": 1})
    assert not o["slip"].any()                                            # no frames -> ToF never flags
    o = replay_py.run(rec.arrays(), {"mode": 3})
    assert not o["slip"].any()
    o = replay_py.run(rec.arrays(), {"fsrConfirmMs": 0, "fsrDropFrac": 0.8})
    assert not o["slip"].any()
    # NaN current force counts as 0 -> full drop on that row
    rows = monitor_timeline(n_mon=300)
    rows[200] = row(1200, 12_000, "MONITOR", (None, 1.0, 0, 0, 0, 0))
    write_rec(os.path.join(TMP, "sig2"), "grasp_1_medium_20261118-100000.csv", rows)
    o = replay_py.run(sgdata.load_dataset(os.path.join(TMP, "sig2"))[0].arrays(), {"fsrConfirmMs": 0})
    assert abs(o["fsr_drop"][200] - 1.0) < 1e-6 and np.nonzero(o["slip"])[0].tolist() == [200]
    print("ok stand-in signals")


def _synthetic():
    if not os.path.isdir(SYN):
        make_synthetic.generate(SYN, seed=1)
    return sgdata.load_dataset(SYN)


def test_subprocess_path():
    recs = _synthetic()
    cmd = f"{sys.executable} {os.path.join(HERE, 'replay_py.py')}"
    sub, inp = se.Replay(cmd), se.Replay("python")
    assert sub.defaults() == inp.defaults()
    assert sub.defaults()["fsrDropFrac"] == 0.3 and sub.defaults()["mode"] == 2
    p = subprocess.run(cmd.split() + ["--param", "nope=1", "--rows", recs[0].path], capture_output=True, text=True)
    assert p.returncode == 2
    params = {"fsrDropFrac": 0.25, "mode": 2, "tofZoneCount": 6}
    for r in [x for x in recs if x.tof is None][:1] + recs[:3]:
        a, b = sub.run(r, params), inp.run(r, params)
        assert np.array_equal(a["slip"], b["slip"]) and np.array_equal(a["active"], b["active"])
        assert np.array_equal(a["tof_zones"], b["tof_zones"]) and np.allclose(a["fsr_drop"], b["fsr_drop"], atol=6e-4)
        assert list(a["reason"]) == list(b["reason"])
    print("ok subprocess replay path")


def test_cpp_agrees():
    if not os.access(se.CPP_TOOL, os.X_OK):
        print("SKIP C++ cross-check: tools/slip_replay/slip_replay not built (make -C tools/slip_replay)")
        return
    recs = _synthetic()
    cpp = se.Replay(se.CPP_TOOL)
    py_d = replay_py.coerce({})
    assert all(abs(float(py_d[k]) - v) < 1e-6 for k, v in cpp.defaults().items()), "defaults differ"
    sets = [{}, {"mode": 0, "fsrDropFrac": 0.2, "fsrConfirmMs": 0, "fsrWindowMs": 500},
            {"mode": 1, "tofZoneCount": 2, "tofConfirmMs": 0, "tofZoneMm": 8, "tofDispMm": 10},
            {"mode": 3, "fuseWindowMs": 300, "fsrMinForceN": 0.3, "holdoffMs": 800, "settleMs": 300}]
    n_ev = 0
    for params in sets:
        for r in recs:
            c = cpp.run(r, params)
            arr = r.arrays()
            p = replay_py.run(arr, params)
            cs, ps = arr["seq"][c["slip"]].tolist(), arr["seq"][p["slip"]].tolist()
            assert cs == ps, f"{r.name} {params}: C++ {cs} vs Python {ps}"
            n_ev += len(cs)
    # byte-identical full output with the defaults (C++ tool vs stand-in CLI)
    for r in recs[:5]:
        args = ["--rows", r.path] + (["--tof", r.tof_path] if r.tof_path else [])
        a = subprocess.run([se.CPP_TOOL] + args, capture_output=True, text=True, check=True).stdout
        b = subprocess.run([sys.executable, os.path.join(HERE, "replay_py.py")] + args, capture_output=True,
                           text=True, check=True).stdout
        assert a == b, f"output differs on {r.name}"
    print(f"ok C++ replay == Python stand-in ({len(recs)} recordings x {len(sets)} param sets, {n_ev} events)")


def test_end_to_end():
    _synthetic()
    out = os.path.join(TMP, "out")
    t0 = time.time()
    best, a = tune_slip.main(["--data", SYN, "--out", out, "--replay", "python"])
    dt = time.time() - t0
    for f in ("report.md", "best_params.json", "config_patch.txt", "results_all_sets.csv"):
        assert os.path.getsize(os.path.join(out, f)) > 0, f
    patch = open(os.path.join(out, "config_patch.txt")).read()
    assert "slipFsrDropFrac=" in patch and "slipHoldoffMs=" in patch and "SYNTHETIC" in patch
    assert "SYNTHETIC DATA" in open(os.path.join(out, "report.md")).read()
    assert a["det_rate"] >= 0.80, a
    assert a["fa_per_min"] <= 0.5, a
    assert a["dist_rate"] <= 0.30, a
    print(f"ok end-to-end synthetic (stand-in, selected params): detection {a['n_det']}/{a['n_slip']}, "
          f"{a['fa_per_min']:.2f} FA/min, disturbances {100 * a['dist_rate']:.0f} %, tune {dt:.0f} s")


if __name__ == "__main__":
    try:
        test_parsing()
        test_episodes()
        test_scoring()
        test_standin_signals()
        test_subprocess_path()
        test_cpp_agrees()
        test_end_to_end()
        print("ALL TESTS PASSED")
    finally:
        shutil.rmtree(TMP, ignore_errors=True)
