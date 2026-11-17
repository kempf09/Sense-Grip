#!/usr/bin/env python3
"""Synthetic recordings in exactly the dashboard download format (plumbing self-test of the pipeline).

    make_synthetic.py <out_dir> [--seed 1]

Covers the 6 demo objects x the protocol trials (docs/RECORDING_PROTOCOL.md §2):
  A quiet hold x2 (30 s, TPE relaxation -10 %/10 s), B disturbances x1 (tap table / tap base / touch object / hand wave,
  each with O ... E), C pull x3 (slow / fast, came free / only until it moves), D twist x3, E slide x3 (elongated only),
  egg: A, B and a gentle C.  Slips: 30-90 % drops within 100-300 ms on 2-4 pads (twist: some pads rise), object zones
  changing 15-60 mm; marks 100-300 ms late; E 0.3-1.2 s after the motion; some objects stay displaced after E.
Plus: one recording with a GAP (rows missing), one without the _tof file, one with a dead pad (f6/r6 empty),
one with a sensor offline for a while, one manual recording (disarmed, jogged: no MONITOR), a partial notes file.

Every value is synthetic: results on this data test the plumbing, never the thresholds.
"""
from __future__ import annotations

import argparse
import datetime as dt
import math
import os
import sys

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay_py  # noqa: E402
import sgdata  # noqa: E402

DT = 10                     # ms per row
FRAME_MS = 22               # one frame record per single-sensor update (3 sensors in turn, ~15 Hz each)

# name, class, shape, elongated, contact pads (0-based), hold force range N, sensor->object mm, object size (w, h)
OBJECTS = [
    ("egg", "delicate", "round", False, [1, 3, 5], (0.20, 0.40), 45, (45, 60)),
    ("tube", "small", "elongated", True, [0, 1, 2, 3, 5], (0.35, 0.9), 35, (25, 120)),
    ("mug", "medium", "round", False, [0, 1, 2, 3, 4, 5], (0.5, 1.6), 60, (85, 95)),
    ("apple", "medium", "round", False, [0, 1, 3, 4, 5], (0.4, 1.3), 55, (75, 75)),
    ("drill", "large", "elongated", True, [0, 1, 2, 3, 4, 5], (0.8, 2.2), 70, (45, 140)),
    ("asym", "medium", "irregular", False, [0, 2, 3, 5], (0.4, 1.5), 50, (70, 60)),
]
CLASS_PROFILE = {"small": (10, 1.0, 200, 5), "medium": (10, 2.0, 260, 5), "large": (10, 4.0, 350, 5),
                 "delicate": (15, 0.4, 150, 3)}     # firmware defaults (config_core.cpp): stand-off, N, current, speed


def ramp(t, t0, dur):
    """0 before t0, cosine ramp to 1 over dur (ms)."""
    x = np.clip((t - t0) / max(dur, 1), 0, 1)
    return 0.5 - 0.5 * np.cos(np.pi * x)


def bump(t, t0, dur):
    x = (t - t0) / max(dur, 1)
    return np.where((x >= 0) & (x <= 1), np.sin(np.pi * np.clip(x, 0, 1)) ** 2, 0.0)


def damped(t, t0, f_hz=22.0, tau=60.0):
    x = t - t0
    return np.where(x >= 0, np.sin(2 * np.pi * f_hz * x / 1000) * np.exp(-np.clip(x, 0, None) / tau), 0.0)


class Scene:
    """Ground truth of one recording: per-pad force multipliers and per-zone offsets as functions of time."""

    def __init__(self, rng, obj, t_mon0):
        self.rng, self.obj, self.t0 = rng, obj, t_mon0
        self.force_terms = []      # (pad mask(6), fn(t)->multiplier delta per pad)
        self.zone_terms = []       # (amp(3,64) mm, fn(t)->0..1)
        self.free_at = None        # object gone (zones far, no measurement, forces 0)
        self.marks = []            # (t_ms, type)


def build_zones(rng, obj):
    """Static ToF truth: per-sensor 8x8 zones; object block near, a few near background zones, far/no-target rest."""
    _, _, _, elong, _, _, d_obj, (w, h) = obj
    base = np.zeros((3, 8, 8))
    is_obj = np.zeros((3, 8, 8), bool)
    near_bg = np.zeros((3, 8, 8), bool)
    for s in range(3):
        far = rng.uniform(600, 1600, (8, 8))
        far[rng.random((8, 8)) < 0.3] = 0                      # no target
        base[s] = far
        rows = (1, 7) if elong else (2, 6)
        cols = (2, 6) if w < 60 else (1, 7)
        rr, cc = np.mgrid[0:8, 0:8]
        blk = (rr >= rows[0]) & (rr < rows[1]) & (cc >= cols[0]) & (cc < cols[1])
        curv = 6.0 * ((cc - 3.5) ** 2 + (rr - 3.5) ** 2) / 12.0
        base[s][blk] = d_obj + s * 4 + curv[blk]
        is_obj[s] = blk
        nb = (~blk) & (rr == 7)                                 # palm edge visible at 200-330 mm
        base[s][nb] = rng.uniform(200, 330, nb.sum())
        near_bg[s] = nb
    sysoff = 1 + rng.uniform(-0.05, 0.05, (3, 8, 8))          # systematic per-zone error
    return base.reshape(3, 64), is_obj.reshape(3, 64), near_bg.reshape(3, 64), sysoff.reshape(3, 64)


def plan_monitor(sc: Scene, trial: str, rep: int, gentle: bool):
    """Fill the scene with the trial's actions; returns MONITOR duration (ms)."""
    rng, t0 = sc.rng, sc.t0
    contact = sc.obj[4]
    if trial == "A":
        return 30000
    if trial == "B":
        t = t0 + 3000
        kinds = ["tap_table", "tap_base", "touch", "hand", "hand", "touch"]
        for k in kinds:
            onset = t + rng.uniform(0, 400)
            sc.marks.append((onset + rng.uniform(100, 300), "other"))
            dur = rng.uniform(150, 500)
            if k in ("tap_table", "tap_base"):
                amp = rng.uniform(0.04, 0.10)
                pads = np.isin(np.arange(6), contact)
                sc.force_terms.append((pads * amp, lambda tt, o=onset: damped(tt, o)))
                if k == "tap_base":
                    m = np.zeros((3, 64))
                    zz = rng.choice(np.nonzero(sc.is_obj.ravel())[0], 2, replace=False)
                    m.ravel()[zz] = rng.uniform(-6, 6, 2)
                    sc.zone_terms.append((m, lambda tt, o=onset: bump(tt, o, 120)))
            elif k == "touch":
                amp = rng.uniform(-0.10, 0.10, 6) * np.isin(np.arange(6), contact)
                sc.force_terms.append((amp, lambda tt, o=onset, d=dur: bump(tt, o, d + 200)))
                m = np.where(sc.is_obj, rng.uniform(-8, 8, (3, 64)), 0) * (rng.random((3, 64)) < 0.3)
                sc.zone_terms.append((m, lambda tt, o=onset, d=dur: bump(tt, o, d + 200)))
            else:                                               # hand passing through the view
                nb = np.nonzero(sc.near_bg.ravel())[0]
                k_z = int(rng.integers(1, 7))
                zz = rng.choice(nb, min(k_z, len(nb)), replace=False)
                m = np.zeros((3, 64))
                m.ravel()[zz] = -sc.base.ravel()[zz] + rng.uniform(100, 180, len(zz))
                hold = rng.uniform(80, 260)
                sc.zone_terms.append((m, lambda tt, o=onset, d=hold: ((tt >= o) & (tt < o + d)).astype(float)))
            sc.marks.append((onset + dur + rng.uniform(300, 1000), "end"))
            t = onset + rng.uniform(4500, 5500)
        return int(t - t0 + 1500)
    # slips
    typ = {"C": "pull", "D": "twist", "E": "slide"}[trial]
    onset = t0 + rng.uniform(3000, 3800)
    sc.marks.append((onset + rng.uniform(100, 300), typ))
    motion = rng.uniform(100, 300)
    weak = gentle or (trial == "C" and rep == 3)
    free = trial == "C" and rep == 2 and not gentle
    cp = list(contact)
    n_drop = int(rng.integers(2, min(4, len(cp)) + 1))
    drop_pads = rng.choice(cp, n_drop, replace=False)
    amp = np.zeros(6)
    lo, hi = (0.35, 0.5) if weak else (0.4, 0.9)
    amp[drop_pads] = -rng.uniform(lo, hi, n_drop)
    if typ == "twist":
        rest = [p for p in cp if p not in drop_pads]
        if rest:
            amp[rest] = rng.uniform(0.1, 0.4, len(rest))
    sc.force_terms.append((amp, lambda tt, o=onset, d=motion: ramp(tt, o, d)))
    # object zones move
    dmm = rng.uniform(15, 25) if weak else rng.uniform(20, 60)
    m = np.zeros((3, 64))
    if typ == "pull":
        m[sc.is_obj] = dmm * rng.uniform(0.8, 1.1, sc.is_obj.sum())
    elif typ == "twist":
        m[sc.is_obj] = dmm * rng.choice([-1, 1], sc.is_obj.sum()) * rng.uniform(0.6, 1.0, sc.is_obj.sum())
        m[~sc.is_obj] = 0
    else:                                                       # slide: edge zones lose / gain the object
        obj_idx = np.nonzero(sc.is_obj.ravel())[0]
        sel = rng.choice(obj_idx, max(6, len(obj_idx) // 3), replace=False)
        m.ravel()[sel] = dmm * rng.uniform(0.7, 1.2, len(sel))
    sc.zone_terms.append((m, lambda tt, o=onset, d=motion: ramp(tt, o, d + 150)))
    t_end_motion = onset + motion + rng.uniform(300, 1500)
    if free:
        sc.free_at = onset + motion + rng.uniform(100, 400)
        t_end_motion = sc.free_at + rng.uniform(200, 600)
    sc.marks.append((t_end_motion + rng.uniform(100, 300), "end"))
    displaced_after = free or rng.random() < 0.5
    if not displaced_after:                                     # operator lets go: object settles back
        back = sc.marks[-1][0] + rng.uniform(0, 200)
        sc.force_terms.append((-amp * 0.9, lambda tt, o=back: ramp(tt, o, 300)))
        sc.zone_terms.append((-m * 0.95, lambda tt, o=back: ramp(tt, o, 300)))
    return int(sc.marks[-1][0] - t0 + rng.uniform(1000, 3000))


def make_recording(rng, obj, trial, rep, num, t_start, seq0, opts):
    name_obj, cls, shape, elong, contact, frange, d_obj, (w, h) = obj
    gentle = name_obj == "egg"
    manual = opts.get("manual", False)
    # timeline (ms, absolute millis)
    T = {"IDLE0": t_start, "START": t_start + 1000, "PRE_TOUCH": t_start + 1100, "GRASP": t_start + 2300,
         "MONITOR": t_start + 2900}
    sc = Scene(rng, obj, T["MONITOR"])
    sc.base, sc.is_obj, sc.near_bg, sc.sysoff = build_zones(rng, obj)
    mon = plan_monitor(sc, trial, rep, gentle)
    T["OPEN"] = T["MONITOR"] + mon
    T["IDLE1"] = T["OPEN"] + 600
    t_stop = T["IDLE1"] + 1000
    t = np.arange(t_start, t_stop, DT, dtype=np.int64)
    n = len(t)
    seq = seq0 + np.arange(n)
    order = ["IDLE0", "START", "PRE_TOUCH", "GRASP", "MONITOR", "OPEN", "IDLE1"]
    st_names = ["IDLE", "START", "PRE_TOUCH", "GRASP", "MONITOR", "OPEN", "IDLE"]
    state = np.empty(n, object)
    for k, s in zip(order, st_names):
        state[t >= T[k]] = s
    if manual:
        state[:] = "IDLE"
    # forces
    F0 = np.zeros(6)
    F0[contact] = rng.uniform(*frange, len(contact))
    close = ramp(t, T["GRASP"], 500) * (1 - ramp(t, T["OPEN"], 400))
    tm = np.clip(t - T["MONITOR"], 0, None) / 1000.0
    relax = 1 - 0.25 * (1 - np.exp(-tm / 25.0))                # -1 %/s initially (-10 %/10 s)
    mult = np.ones((n, 6))
    for amp, fn in sc.force_terms:
        mult += np.outer(fn(t), amp)
    if sc.free_at is not None:
        mult *= (1 - ramp(t, sc.free_at, 200))[:, None]
    truth = np.clip(F0[None, :] * close[:, None] * relax[:, None] * mult, 0, None)
    raw = truth + rng.normal(0, 1, (n, 6)) * (0.01 + 0.03 * truth)
    raw = np.where(truth > 0.005, raw, np.abs(rng.normal(0, 0.003, (n, 6))) * (rng.random((n, 6)) < 0.1))
    raw = np.clip(raw, 0, None)
    filt = np.zeros_like(raw)
    acc = raw[0].copy()
    for i in range(n):                                          # EMA alpha 0.3 (dashboard filter "EMA")
        acc = acc + 0.3 * (raw[i] - acc)
        filt[i] = acc
    # servo
    open_pos, close_pos = 2048, 2048 + 1500 - 6 * w
    pos = open_pos + (close_pos - open_pos) * ramp(t, T["PRE_TOUCH"], 1500) * (1 - ramp(t, T["OPEN"], 400))
    speed = np.gradient(pos) * 100 / 1.0
    cur = np.where(np.isin(state, ["GRASP", "PRE_TOUCH"]), 150 + 40 * close, 0) + rng.normal(0, 3, n)
    load = np.clip(cur / 10, 0, None)
    contacts = np.stack([(filt[:, 2 * f] >= 0.1) | (filt[:, 2 * f + 1] >= 0.1) for f in range(3)], axis=1)
    # frames
    ft = np.arange(t_start + int(rng.uniform(0, FRAME_MS)), t_stop - DT, FRAME_MS)
    ft = np.sort(ft + rng.integers(-3, 4, len(ft)))
    fing = 1 + 0.6 * (1 - ramp(ft, T["PRE_TOUCH"], 1500) * (1 - ramp(ft, T["OPEN"], 400)))   # fingers open: farther
    Z = np.repeat(sc.base[None], len(ft), 0) * np.where(sc.is_obj[None], fing[:, None, None], 1.0)
    for amp, fn in sc.zone_terms:
        Z += fn(ft)[:, None, None] * amp[None]
    if sc.free_at is not None:
        gone = ft >= sc.free_at + 150
        far = np.where(rng.random((3, 64)) < 0.3, 0, rng.uniform(700, 1500, (3, 64)))
        Z[gone] = np.where(sc.is_obj[None], far[None], Z[gone])
    noisy = Z * sc.sysoff[None] + rng.normal(0, 1, Z.shape) * (1.5 + 0.01 * Z)
    noisy = np.where(Z <= 0, 0, noisy)
    noisy[rng.random(noisy.shape) < 0.003] = 0                 # single-zone dropouts
    zones = np.clip(np.rint(noisy), 0, 4000).astype(int)
    rec_i = np.arange(len(ft))                                  # record j refreshes sensor j % 3 only
    for s in range(3):
        fresh = rec_i[rec_i % 3 == s]
        if len(fresh):
            k = np.maximum(np.searchsorted(fresh, rec_i, side="right") - 1, 0)
            zones[:, s] = zones[fresh[k], s]
    fvalid = np.full(len(ft), 7)
    if opts.get("sensor_off"):
        off = (ft > T["MONITOR"] + 8000) & (ft < T["MONITOR"] + 12000)
        fvalid[off] = 3
        zones[off, 2, :] = 0
    row_idx = np.searchsorted(t, ft, side="left")              # next main row written after the frame
    keep = row_idx < n
    ft, zones, fvalid, row_idx = ft[keep], zones[keep], fvalid[keep], row_idx[keep]
    tof_seq_at_row = np.searchsorted(ft, t, side="right")      # frames measured so far
    # smoothed measurement
    objmask = sc.is_obj
    disp_frames = (Z[keep] - (np.repeat(sc.base[None], keep.sum(), 0) *
                   np.where(objmask[None], fing[keep][:, None, None], 1.0)))[:, objmask].mean(axis=1)
    disp_rows = np.interp(t, ft, disp_frames) if len(ft) else np.zeros(n)
    meas_ok = (t >= T["START"]) & ((sc.free_at is None) | (t < (sc.free_at or 0) + 100))
    rng_mm = d_obj + 40 + 0.8 * disp_rows + rng.normal(0, 0.4, n)
    width = w + rng.normal(0, 0.3, n)
    height = h + 0.3 * disp_rows + rng.normal(0, 0.4, n)
    rads = np.stack([w / 2 + rng.normal(0, 0.3, n) + 0.2 * disp_rows * (s + 1) / 3 for s in range(3)], 1)
    # events
    events = np.full(n, "", object)
    prev = None
    for i in range(n):
        if state[i] != prev and not manual:
            events[i] = f"state:{state[i]}"
        prev = state[i]
    if not manual:
        k = int(np.searchsorted(t, T["MONITOR"])) - 1
        events[k] = "grasp_reached"
        for f in range(3):
            on = np.nonzero(contacts[:, f] & (t >= T["GRASP"]))[0]
            if len(on):
                j = on[0]
                while j < n and events[j]:
                    j += 1
                if j < n:
                    events[j] = f"contact:F{f + 1}:{['proximal', 'distal', 'both'][int(rng.integers(0, 3))]}"
    else:
        contacts[:] = False
    for tm, typ in sc.marks:
        j = int(np.searchsorted(t, tm))
        while j < n and events[j]:
            j += 1
        if j < n:
            events[j] = f"mark:{typ}"
    # rows -> text
    stamp = dt.datetime(2026, 11, 18, 10, 0, 0) + dt.timedelta(milliseconds=int(t_start))
    kind = "manual" if manual else "grasp"
    nm = f"{kind}_{num}" + ("" if manual else f"_{cls}") + f"_{stamp:%Y%m%d-%H%M%S}.csv"
    dead = opts.get("dead_pad")
    keep_rows = np.ones(n, bool)
    if opts.get("gap"):
        g0 = int(np.searchsorted(t, T["MONITOR"] + 12000))
        keep_rows[g0:g0 + 45] = False

    def f3(x):
        return f"{x:.3f}"

    def f1(x):
        return f"{x:.1f}"

    lines = []
    step = np.where(state == "PRE_TOUCH", np.where(t < T["PRE_TOUCH"] + 1000, "approach", "slow"), "-")
    for i in np.nonzero(keep_rows)[0]:
        fs = [("" if dead == p else f3(filt[i, p])) for p in range(6)]
        rs = [("" if dead == p else f3(raw[i, p])) for p in range(6)]
        mv = meas_ok[i]
        rad = [f1(rads[i, s]) if mv else "" for s in range(3)]
        if mv and name_obj == "asym":
            rad[1] = ""                                         # one sensor does not see the asymmetric object
        armed = 0 if manual else int(state[i] not in ("IDLE",) or t[i] < T["START"])
        lines.append(",".join([
            str(seq[i]), str(t[i]), state[i], step[i], str(armed), str(int(round(pos[i]))), str(int(round(speed[i]))),
            str(int(round(cur[i]))), f1(load[i]), *fs, *rs, *[str(int(c)) for c in contacts[i]],
            str(int(tof_seq_at_row[i]) % 65536), str(int(mv)), f1(width[i]) if mv else "", f1(rng_mm[i]) if mv else "",
            f1(height[i]) if mv else "", *rad, ("" if manual else cls), (shape if mv else ""), events[i]]))
    prof = CLASS_PROFILE[cls]
    started = (dt.datetime(2026, 11, 18, 9, 0, 0) + dt.timedelta(milliseconds=int(t_start))).isoformat(timespec="milliseconds") + "Z"
    head = [f"# SenseGrip recording {nm}", "# firmware: synthetic (make_synthetic.py) phase4-rec",
            f"# started: {started} | rows: {int(keep_rows.sum())} | result: {'manual stop' if manual else 'grasped'}"
            + (" | GAP: rows missing" if opts.get("gap") else "")]
    if not manual:
        head.append(f"# class profile {cls}: stand-off {prof[0]} mm, target {prof[1]} N, max current {prof[2]} "
                    f"x6.5 mA, grasp speed {prof[3]}")
    head.append("# slip: " + ",".join(f"slip{k[0].upper()}{k[1:]}={float(v):g}" for k, v in replay_py.DEFAULTS.items()))
    head += ["# FSR filter: EMA window 5 alpha 0.3 | contact force 0.25 N | max force 8 N",
             "# units: forces N (f=filtered, r=raw, FSR1..6), pos servo counts, current mA, load %, sizes mm"]
    main = "\n".join(head + [",".join(sgdata.MAIN_HEADER)] + lines) + "\n"
    tof_lines = [f"# SenseGrip raw SEN0628 frames for {nm} | mm per zone, raw zone index (row-major as read), 0 or "
                 f">=4000 = no target | row_seq links to seq in the main file", ",".join(sgdata.TOF_HEADER)]
    for j in range(len(ft)):
        tof_lines.append(f"{j + opts.get('tof_seq0', 0)},{ft[j]},{seq[row_idx[j]]},{int(round(pos[row_idx[j]]))},"
                         f"{fvalid[j]}," + ",".join(map(str, zones[j].ravel())))
    tof = "\n".join(tof_lines) + "\n"
    return nm, main, (None if opts.get("no_tof") else tof), t_stop, seq0 + n, len(ft)


def add_logged_slip_events(path, rng):
    """What the firmware (default settings) would have logged: `slip:<reason>` 1..3 rows after the detector fired."""
    rec = sgdata.load_recording(path)
    out = replay_py.run(rec.arrays(), {})
    with open(path) as fh:
        lines = fh.read().split("\n")
    h = next(i for i, ln in enumerate(lines) if ln.startswith("seq,"))
    for i in np.nonzero(out["slip"])[0]:
        for k in range(int(rng.integers(1, 4)), 6):
            j = h + 1 + i + k
            if j < len(lines) and lines[j] and lines[j].endswith(","):
                lines[j] += "slip:" + replay_py.REASONS[out["reason"][i]]
                break
    with open(path, "w", newline="\n") as fh:
        fh.write("\n".join(lines))


def generate(out_dir: str, seed: int = 1) -> list[str]:
    rng = np.random.default_rng(seed)
    os.makedirs(out_dir, exist_ok=True)
    plan = []
    for obj in OBJECTS:
        trials = [("A", 1), ("A", 2), ("B", 1)] + [("C", r) for r in (1, 2, 3)]
        if obj[0] != "egg":
            trials += [("D", r) for r in (1, 2, 3)]
            if obj[3]:
                trials += [("E", r) for r in (1, 2, 3)]
        plan += [(obj, tr, rep) for tr, rep in trials]
    special = {("mug", "A", 2): {"gap": True}, ("apple", "D", 3): {"no_tof": True},
               ("tube", "A", 1): {"dead_pad": 4}, ("drill", "B", 1): {"sensor_off": True}}
    trial_names = {"A": "A quiet", "B": "B disturbances", "C": "C pull", "D": "D twist", "E": "E slide"}
    t_cur, seq_cur, tseq, num = 600_000, 60_000, 5000, 0
    notes = ["file,object,trial,notes"]
    names = []
    for i, (obj, tr, rep) in enumerate(plan + [(OBJECTS[2], "C", 1)]):
        num += 1
        manual = i == len(plan)
        opts = dict(special.get((obj[0], tr, rep), {}))
        opts["manual"] = manual
        opts["tof_seq0"] = tseq
        nm, main, tof, t_stop, seq_end, nfr = make_recording(rng, obj, tr, rep, num, t_cur, seq_cur, opts)
        with open(os.path.join(out_dir, nm), "w", newline="\n") as fh:
            fh.write(main)
        if tof is not None:
            with open(os.path.join(out_dir, nm[:-4] + "_tof.csv"), "w", newline="\n") as fh:
                fh.write(tof)
        if not manual:
            add_logged_slip_events(os.path.join(out_dir, nm), rng)
        names.append(nm)
        gap_s = int(rng.uniform(20_000, 60_000))
        seq_cur = seq_end + gap_s // DT
        t_cur = t_stop + gap_s
        tseq += nfr + gap_s // FRAME_MS
        note = {1: "slow", 2: "fast, came free", 3: "until it moves"}.get(rep, "") if tr == "C" else ""
        if manual:
            notes.append(f"{nm},mug,C pull,manual (jogged closed)")
        elif i % 19 != 4:                                         # partial notes: some files missing
            obj_name = "asymmetric" if obj[0] == "asym" else obj[0]
            notes.append(f"{nm},{obj_name},{trial_names[tr]} {rep},{chr(34) + note + chr(34) if ',' in note else note}")
    with open(os.path.join(out_dir, "notes.csv"), "w", newline="\n") as fh:
        fh.write("\n".join(notes) + "\n")
    return names


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_dir")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    names = generate(a.out_dir, a.seed)
    print(f"wrote {len(names)} synthetic recordings (+ _tof files, notes.csv) to {a.out_dir}")


if __name__ == "__main__":
    main()
