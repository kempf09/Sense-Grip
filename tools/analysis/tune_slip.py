#!/usr/bin/env python3
"""Grid-search the slip detector parameters through the replay tool and write report.md, best_params.json,
config_patch.txt (spec 2026-11-17-phase4b §7).

    tune_slip.py --data <recordings dir> --out <out dir> [--replay auto|python|"<command>"]
                 [--max-fa-per-min 0.5] [--max-dist-frac 0.30] [--workers N] [--quick]

Grid (staged so ~60 recordings stay well under 10 minutes; every number comes from the replay tool, nothing of the
detector is re-implemented here):
  A  FSR only (mode 0):  fsrDropFrac {0.2,0.25,0.3,0.4,0.5,0.6} x fsrWindowMs {150,300,500}
                         x fsrConfirmMs {0,30,60} x fsrMinForceN {0.15,0.3}                        = 108 sets
  B  ToF only (mode 1):  tofZoneMm {8,12,20,30} x tofZoneCount {2,4,6,10} x tofConfirmMs {0,150,300,500}
                         + tofDispMm {10,20} at the default zone settings                           =  66 sets
  C  fusion, per context (all data, and each leave-one-object-out training set, so the held-out object never
     influences its candidates): the 3 best + the most sensitive (false alarms <= 4x the limit) sets of A and of B,
     crossed: mode 2 (either), mode 3 (both) x fuseWindowMs {300,600,1000}                          <= 64 sets
  + the current defaults.  Fixed (designer's calls, spec §8): settleMs, holdoffMs, tofNearMm, tofZoneFrac.
Selection: maximise slip detection rate subject to quiet false alarms <= --max-fa-per-min and the fraction of
disturbance episodes with an event <= --max-dist-frac; ties -> lower mean latency, then fewer false alarms.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import multiprocessing as mp
import os
import sys
import time

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import sgdata  # noqa: E402
import slipeval as se  # noqa: E402

FULL_GRID = {
    "A": {"fsrDropFrac": [0.2, 0.25, 0.3, 0.4, 0.5, 0.6], "fsrWindowMs": [150, 300, 500],
          "fsrConfirmMs": [0, 30, 60], "fsrMinForceN": [0.15, 0.3]},
    "B": {"tofZoneMm": [8, 12, 20, 30], "tofZoneCount": [2, 4, 6, 10], "tofConfirmMs": [0, 150, 300, 500]},
    "B_disp": [10, 20],
    "fuse": [300, 600, 1000], "k_best": 3,
}
QUICK_GRID = {
    "A": {"fsrDropFrac": [0.25, 0.3, 0.4], "fsrWindowMs": [300], "fsrConfirmMs": [30], "fsrMinForceN": [0.15]},
    "B": {"tofZoneMm": [12, 20], "tofZoneCount": [4, 6], "tofConfirmMs": [150]},
    "B_disp": [], "fuse": [500], "k_best": 1,
}
FSR_KEYS = ["fsrDropFrac", "fsrWindowMs", "fsrMinForceN", "fsrConfirmMs"]
TOF_KEYS = ["tofNearMm", "tofZoneMm", "tofZoneFrac", "tofZoneCount", "tofConfirmMs", "tofDispMm"]

_RECS: list = []
_REPLAY: se.Replay | None = None
_TOL = sgdata.TOL_S
_GRID = FULL_GRID


def _key(p: dict) -> tuple:
    return tuple((k, float(p[k])) for k in se.PARAM_NAMES)


def _work(task):
    pi, params, ri = task
    rec = _RECS[ri]
    return pi, ri, se.score(rec, _REPLAY.run(rec, params), _TOL)


class Evaluator:
    """Runs param sets over all recordings (cached), in parallel, deterministic order."""

    def __init__(self, recs, replay, workers):
        self.recs, self.replay, self.workers = recs, replay, workers
        self.cache: dict[tuple, list] = {}
        self.params: dict[tuple, dict] = {}
        self.runs = 0

    def evaluate(self, sets: list[dict]):
        new = []
        for p in sets:
            k = _key(p)
            if k not in self.cache and k not in self.params:
                self.params[k] = p
                new.append(k)
        if not new:
            return
        tasks = [(i, self.params[k], ri) for i, k in enumerate(new) for ri in range(len(self.recs))]
        res = [[None] * len(self.recs) for _ in new]
        if self.workers > 1:
            with mp.get_context("fork").Pool(self.workers) as pool:
                for pi, ri, sc in pool.imap(_work, tasks, chunksize=max(1, len(self.recs) // 4)):
                    res[pi][ri] = sc
        else:
            for t in tasks:
                pi, ri, sc = _work(t)
                res[pi][ri] = sc
        for i, k in enumerate(new):
            self.cache[k] = res[i]
        self.runs += len(tasks)

    def agg(self, p: dict, idx: list[int]) -> dict:
        sc = self.cache[_key(p)]
        return se.aggregate([sc[i] for i in idx])

    def scores(self, p: dict, idx: list[int]) -> list:
        sc = self.cache[_key(p)]
        return [sc[i] for i in idx]


def stage_sets(defaults: dict, grid: dict) -> tuple[list, list]:
    A, B = [], []
    ka = list(grid["A"].keys())
    for vals in itertools.product(*grid["A"].values()):
        p = dict(defaults, mode=0, **dict(zip(ka, vals)))
        A.append(p)
    kb = list(grid["B"].keys())
    for vals in itertools.product(*grid["B"].values()):
        B.append(dict(defaults, mode=1, **dict(zip(kb, vals))))
    for d in grid["B_disp"]:
        B.append(dict(defaults, mode=1, tofDispMm=d))
    return A, B


def ranked(ev: Evaluator, sets: list, idx: list, lim) -> list:
    return sorted(sets, key=lambda p: (se.rank_key(ev.agg(p, idx), *lim), _key(p)))


def shortlist(ev, sets, idx, lim, k):
    r = ranked(ev, sets, idx, lim)
    out = r[:k]
    sens = [p for p in sets if ev.agg(p, idx)["fa_per_min"] <= 4 * lim[0]]
    if sens:
        best = min(sens, key=lambda p: (-np.nan_to_num(ev.agg(p, idx)["det_rate"]), ev.agg(p, idx)["lat_mean_ms"], _key(p)))
        if all(_key(best) != _key(q) for q in out):
            out.append(best)
    return out


def fusion_sets(fsr_short, tof_short, defaults, grid):
    out = []
    for a in fsr_short:
        for b in tof_short:
            base = dict(defaults)
            base.update({k: a[k] for k in FSR_KEYS})
            base.update({k: b[k] for k in TOF_KEYS})
            out.append(dict(base, mode=2))
            for f in grid["fuse"]:
                out.append(dict(base, mode=3, fuseWindowMs=f))
    return out


def choose(ev, A, B, defaults, idx, lim, grid):
    """Staged search on the recordings `idx`; returns (best params, candidate list)."""
    fs = shortlist(ev, A, idx, lim, grid["k_best"])
    ts = shortlist(ev, B, idx, lim, grid["k_best"])
    C = fusion_sets(fs, ts, defaults, grid)
    ev.evaluate(C)
    cands = A + B + C + [defaults]
    return ranked(ev, cands, idx, lim)[0], cands


# ---------------------------------------------------------------------------------------------------------------
def fmt(x, nd=2, pct=False):
    if x is None or (isinstance(x, float) and (math.isnan(x))):
        return "–"
    if isinstance(x, float) and math.isinf(x):
        return "inf"
    if pct:
        return f"{100 * x:.0f} %"
    return f"{x:.{nd}f}" if isinstance(x, float) else str(x)


def table(head, rows):
    out = ["| " + " | ".join(head) + " |", "|" + "|".join("---" for _ in head) + "|"]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def metric_row(label, a):
    return [label, f"{a['n_det']}/{a['n_slip']}", fmt(a["det_rate"], pct=True), fmt(a["lat_median_ms"], 0),
            fmt(a["lat_p90_ms"], 0), f"{a['n_fa']} / {a['quiet_min']:.1f} min", fmt(a["fa_per_min"]),
            f"{a['n_dist_trig']}/{a['n_dist']}", fmt(a["dist_rate"], pct=True), a["post_slip_fa"]]


METRIC_HEAD = ["", "slips detected", "rate", "latency median ms", "latency p90 ms", "quiet false alarms",
               "FA/min", "disturbances with event", "dist. rate", "FA ≤10 s after a slip"]


def param_summary(p, defaults=None):
    mode = int(p["mode"])
    s = ["fsr", "tof", "either", "both"][mode] if mode in (0, 1, 2, 3) else f"mode {mode}"
    parts = [f"mode {mode} ({s})"]
    if mode != 1:
        parts.append(f"drop {p['fsrDropFrac']:g} / {p['fsrWindowMs']:g} ms / confirm {p['fsrConfirmMs']:g} ms / "
                     f"min {p['fsrMinForceN']:g} N")
    if mode != 0:
        parts.append(f"zones ≥{p['tofZoneCount']:g} × {p['tofZoneMm']:g} mm for {p['tofConfirmMs']:g} ms"
                     + (f" / disp {p['tofDispMm']:g} mm" if p["tofDispMm"] > 0 else ""))
    if mode == 3:
        parts.append(f"fuse {p['fuseWindowMs']:g} ms")
    return "; ".join(parts)


LOG_LAG_ROWS = (1, 5)   # firmware writes the slip event 1..5 rows after the detector fired (event FIFO)


def logged_vs_replay(recs, replay, defaults):
    """Firmware-logged slip events against the replay with the settings recorded in the file (`# slip:` comment,
    else the defaults). Also collects the replay tool's `warning:` lines. Returns (summary, per-recording rows)."""
    tot = {"logged": 0, "matched": 0, "replayed": 0, "files": 0}
    per, warns = [], {}
    for r in recs:
        if not r.has_monitor():
            continue
        params = dict(defaults)
        params.update({k: v for k, v in r.slip_settings.items() if k in params})
        out = replay.run(r, params)
        if out.get("warnings"):
            warns[r.name] = out["warnings"]
        logged = np.nonzero(r.rows["event"].str.startswith("slip:").to_numpy())[0]
        rep = np.nonzero(out["slip"])[0]
        if not len(logged):
            continue
        m = sum(1 for i in logged if any(LOG_LAG_ROWS[0] <= i - j <= LOG_LAG_ROWS[1] for j in rep))
        tot["logged"] += len(logged)
        tot["matched"] += m
        tot["replayed"] += len(rep)
        tot["files"] += 1
        per.append([r.name, "file" if r.slip_settings else "defaults", len(logged), len(rep), m])
    return tot, per, warns


def write_outputs(args, recs, ev, defaults, best, folds, A, B, cands_all, runtime, plots_rel="plots"):
    out = args.out
    lim = (args.max_fa_per_min, args.max_dist_frac)
    idx_all = list(range(len(recs)))
    synthetic = any(r.synthetic for r in recs)
    a_best, a_def = ev.agg(best, idx_all), ev.agg(defaults, idx_all)
    objects = sorted({r.object for r in recs})
    changed = {k: best[k] for k in se.PARAM_NAMES if float(best[k]) != float(defaults[k])}
    clean = {k: (int(v) if float(v).is_integer() and k not in ("fsrDropFrac", "fsrMinForceN", "tofZoneFrac", "tofDispMm")
                 else float(v)) for k, v in best.items()}
    per_obj = {}
    for o in objects:
        ii = [i for i, r in enumerate(recs) if r.object == o]
        per_obj[o] = {"best": ev.agg(best, ii), "defaults": ev.agg(defaults, ii)}
    meets = se.feasible(a_best, *lim)
    js = {"params": clean, "changed_vs_defaults": changed, "meets_constraints": meets,
          "constraints": {"max_fa_per_min": lim[0], "max_dist_frac": lim[1], "tolerance_s": args.tol},
          "metrics": a_best, "metrics_defaults": a_def, "per_object": {o: v["best"] for o, v in per_obj.items()},
          "per_slip_type": se.by_slip_type(ev.scores(best, idx_all)),
          "leave_one_object_out": [{k: v for k, v in f.items() if k != "params"} | {"params": f["params"]} for f in folds],
          "replay": ev.replay.label, "synthetic": synthetic, "n_recordings": len(recs)}
    with open(os.path.join(out, "best_params.json"), "w") as fh:
        json.dump(js, fh, indent=2, sort_keys=True, default=float)
    with open(os.path.join(out, "config_patch.txt"), "w") as fh:
        fh.write("# GripConfig slip* fields chosen by tools/analysis/tune_slip.py; name=value, '#' lines are comments\n")
        if synthetic:
            fh.write("# SYNTHETIC DATA: plumbing test only, do NOT apply these values to the gripper\n")
        if not meets:
            fh.write("# WARNING: no evaluated parameter set met the false-alarm limits; see report.md\n")
        for k in se.PARAM_NAMES:
            v = clean[k]
            fh.write(f"slip{k[0].upper()}{k[1:]}={v:g}\n" if isinstance(v, float) else f"slip{k[0].upper()}{k[1:]}={v}\n")
    # results table of every evaluated set (all data)
    keys = list(ev.cache.keys())
    rows = sorted(keys, key=lambda k: (se.rank_key(ev.agg(ev.params[k], idx_all), *lim), k))
    with open(os.path.join(out, "results_all_sets.csv"), "w") as fh:
        fh.write(",".join(se.PARAM_NAMES) + ",det_rate,lat_mean_ms,fa_per_min,dist_rate,n_fa,n_det,n_slip\n")
        for k in rows:
            a = ev.agg(ev.params[k], idx_all)
            fh.write(",".join(f"{v:g}" for _, v in k) + f",{a['det_rate']:.4f},{a['lat_mean_ms']:.1f},"
                     f"{a['fa_per_min']:.4f},{a['dist_rate']:.4f},{a['n_fa']},{a['n_det']},{a['n_slip']}\n")
    # ------------------------------------------------------------------------ report.md
    L = ["# Slip detector tuning report", ""]
    if synthetic:
        L += ["> **SYNTHETIC DATA — plumbing test only.** These recordings were generated by "
              "`tools/analysis/make_synthetic.py`. The numbers below say the pipeline works; they are **not** tuned "
              "thresholds and must not be applied to the gripper (spec §7).", ""]
    eps = [e for r in recs for e in r.episodes]
    uns = [e for e in eps if not e.scorable]
    n_manual = sum(r.kind == "manual" for r in recs)
    L += ["## Data", "",
          f"- Folder: `{os.path.abspath(args.data)}` — {len(recs)} recordings "
          f"({len(recs) - n_manual} grasp, {n_manual} manual), objects: {', '.join(objects)}.",
          f"- Replay: **{ev.replay.label}**.",
          f"- MONITOR time {sum(r.monitor_s() for r in recs) / 60:.1f} min; quiet (detector armed, outside every "
          f"episode ±{args.tol:g} s) {a_def['quiet_min']:.1f} min.",
          f"- Episodes: {sum(e.kind == 'slip' for e in eps)} slip "
          f"({', '.join(f'{t} {sum(e.type == t for e in eps)}' for t in sgdata.SLIP_TYPES)}), "
          f"{sum(e.kind == 'disturbance' for e in eps)} disturbance; {len(uns)} not scorable (mark outside MONITOR).",
          f"- Gaps: {sum(1 for r in recs if r.gap_rows or r.gap_flag)} recordings "
          f"({sum(r.gap_rows for r in recs)} rows missing in total). Without `_tof` file: "
          f"{sum(1 for r in recs if r.tof is None)}. Object from file name (no notes line): "
          f"{sum(1 for r in recs if not r.notes and r.object == r.cls)}."]
    if sgdata.SKIPPED:
        L += [f"- **Skipped files ({len(sgdata.SKIPPED)}):** " + "; ".join(f"`{f}` ({w})" for f, w in sgdata.SKIPPED)]
    no_mon = [r for r in recs if not r.has_monitor()]
    if no_mon:
        L += [f"- **Not replayable (no MONITOR): {len(no_mon)} recordings** ({sum(r.kind == 'manual' for r in no_mon)} "
              "manual). The detector only runs in MONITOR, so their marks are not scored (not counted as misses). They "
              "are plotted but cannot tune anything; record slips through a real grasp whenever possible."]
    tot, per_log, rwarn = logged_vs_replay(recs, ev.replay, defaults)
    if tot["logged"]:
        L += [f"- Firmware-logged `slip:` events: {tot['logged']} in {tot['files']} recordings; {tot['matched']} match a "
              f"replayed event {LOG_LAG_ROWS[0]}–{LOG_LAG_ROWS[1]} rows earlier (replay with the settings recorded in the file, else "
              f"the defaults; {tot['replayed']} replayed events). Unmatched logged events mean replay and firmware "
              "disagree (or the settings changed during the recording)."]
    sets = {}
    for r in recs:
        if r.slip_settings:
            sets.setdefault(tuple(sorted(r.slip_settings.items())), []).append(r.name)
    if sets:
        L += [f"- Detector settings at recording time (`# slip:` comment): {len(sets)} distinct set(s) in "
              f"{sum(len(v) for v in sets.values())} recordings; {len(recs) - sum(len(v) for v in sets.values())} "
              "without the comment."]
        for k, names in sets.items():
            diff = {a: b for a, b in k if a in defaults and float(b) != float(defaults[a])}
            L += [f"  - {len(names)} recordings: " + (", ".join(f"{a}={b:g}" for a, b in diff.items()) + " (rest = defaults)"
                                                      if diff else "all defaults")]
    L += ["", "## Selection", "",
          f"Maximise slip detection (event within [mark − {args.tol:g} s, end + {args.tol:g} s]) subject to quiet false "
          f"alarms ≤ **{lim[0]:g}/min** and disturbances with an event ≤ **{100 * lim[1]:.0f} %**; ties → lower mean "
          f"latency (in {se.LAT_TIE_MS:g} ms bins), then fewer false alarms.",
          f"Evaluated {len(ev.cache)} parameter sets × {len(recs)} recordings = {ev.runs} replays "
          "(grid: see `tune_slip.py` docstring / README). The chosen set is the best of every evaluated set "
          "(row 1 of *Top parameter sets*).", ""]
    if not meets:
        L += ["**No evaluated parameter set meets both limits.** The set shown is the one closest to the limits; "
              "see the top-sets table and the plots before using it.", ""]
    edge = []
    for st, keys in (("A", FSR_KEYS), ("B", TOF_KEYS)):
        if (st == "A" and int(best["mode"]) == 1) or (st == "B" and int(best["mode"]) == 0):
            continue
        for k, vals in _GRID[st].items():
            if len(vals) > 1 and float(best[k]) in (float(min(vals)), float(max(vals))):
                edge.append(f"{k}={best[k]:g}")
    L += ["### Chosen parameters", "", f"`{param_summary(best)}`", ""]
    if edge:
        L += [f"Note: {', '.join(edge)} at the edge of the searched grid — the optimum may lie outside it; widen the "
              "grid in `tune_slip.py` (FULL_GRID) before trusting this value.", ""]
    guessed = sorted({r.object for r in recs if not r.notes and r.object == r.cls and r.cls})
    if guessed:
        L += [f"Note: object taken from the class in the file name for {', '.join(guessed)} (no notes line) — "
              "these appear as separate 'objects' below; add them to the notes file.", ""]
    L += [
          table(["param", "chosen", "default", "firmware field"],
                [[k, f"{best[k]:g}" + (" **(changed)**" if k in changed else ""), f"{defaults[k]:g}",
                  f"slip{k[0].upper()}{k[1:]}"] for k in se.PARAM_NAMES]), "",
          "### Overall", "", table(METRIC_HEAD, [metric_row("chosen", a_best), metric_row("defaults", a_def)]), "",
          "Latency = first event − operator mark; negative is normal (marks are pressed late).", "",
          "### Per object", "",
          table(["object"] + METRIC_HEAD[1:], [[o] + metric_row(o, per_obj[o]["best"])[1:] for o in objects]), "",
          "### Per slip type", ""]
    bt = se.by_slip_type(ev.scores(best, idx_all))
    bd = se.by_slip_type(ev.scores(defaults, idx_all))
    L += [table(["type", "chosen detected", "chosen latency median ms", "defaults detected"],
                [[t, f"{v['n_det']}/{v['n_slip']} ({fmt(v['det_rate'], pct=True)})", fmt(v["lat_median_ms"], 0),
                  f"{bd[t]['n_det']}/{bd[t]['n_slip']}" if t in bd else "–"] for t, v in bt.items()]), ""]
    L += ["## Leave-one-object-out", "",
          "Each row: parameters chosen with the same rule on all *other* objects (own shortlist, no access to the "
          "held-out object), then evaluated on the held-out object.", ""]
    if folds:
        L += [table(["held-out", "slips detected", "rate", "FA/min", "dist. rate", "chosen on the others"],
                    [[f["object"], f"{f['held_out']['n_det']}/{f['held_out']['n_slip']}",
                      fmt(f["held_out"]["det_rate"], pct=True), fmt(f["held_out"]["fa_per_min"]),
                      fmt(f["held_out"]["dist_rate"], pct=True), param_summary(f["params"])] for f in folds]), ""]
        dr = [f["held_out"]["det_rate"] for f in folds if f["held_out"]["n_slip"]]
        fa = [f["held_out"]["fa_per_min"] for f in folds if f["held_out"]["quiet_min"] > 0]
        if dr:
            L += [f"Spread: detection {100 * min(dr):.0f}–{100 * max(dr):.0f} % (mean {100 * np.mean(dr):.0f} %, "
                  f"sd {100 * np.std(dr):.0f} %); false alarms {min(fa):.2f}–{max(fa):.2f}/min. "
                  f"Folds choosing the same mode as the full fit: "
                  f"{sum(int(f['params']['mode']) == int(best['mode']) for f in folds)}/{len(folds)}.", ""]
    else:
        L += ["Not enough objects with scorable slips (need ≥ 2).", ""]
    L += ["## Top parameter sets (all data)", "",
          table(["#", "parameters", "detected", "lat. median", "FA/min", "dist. rate", "meets limits"],
                [[i + 1, param_summary(ev.params[k]), fmt(ev.agg(ev.params[k], idx_all)["det_rate"], pct=True),
                  fmt(ev.agg(ev.params[k], idx_all)["lat_median_ms"], 0), fmt(ev.agg(ev.params[k], idx_all)["fa_per_min"]),
                  fmt(ev.agg(ev.params[k], idx_all)["dist_rate"], pct=True),
                  "yes" if se.feasible(ev.agg(ev.params[k], idx_all), *lim) else "no"] for i, k in enumerate(rows[:12])]),
          "", "Every evaluated set: `results_all_sets.csv`.", ""]
    best_def = {}
    for mode in (0, 1, 2, 3):
        ks = [k for k in rows if int(ev.params[k]["mode"]) == mode]
        if ks:
            best_def[mode] = ks[0]
    L += ["### Best set per fusion mode", "",
          table(["mode", "parameters", "detected", "FA/min", "dist. rate", "FA ≤10 s after a slip"],
                [[m, param_summary(ev.params[k]), fmt(ev.agg(ev.params[k], idx_all)["det_rate"], pct=True),
                  fmt(ev.agg(ev.params[k], idx_all)["fa_per_min"]), fmt(ev.agg(ev.params[k], idx_all)["dist_rate"], pct=True),
                  ev.agg(ev.params[k], idx_all)["post_slip_fa"]] for m, k in best_def.items()]), "",
          "`FA ≤10 s after a slip` counts quiet false alarms shortly after a slip episode: the ToF baseline is only "
          "taken on MONITOR entry (spec §4), so an object left displaced after a slip keeps the ToF feature flagged "
          "and re-fires every holdoff.", ""]
    # per recording
    sc_best = ev.scores(best, idx_all)
    L += ["## Recordings", "", "Plots: one PNG per recording and one overview per object (run `plot_recording.py`).",
          "", " ".join(f"[overview {o}]({plots_rel}/overview_{o}.png)" for o in objects), ""]
    rr = []
    for r, s in zip(recs, sc_best):
        flags = []
        if r.kind == "manual":
            flags.append("manual")
        if r.gap_rows or r.gap_flag:
            flags.append(f"gap {r.gap_rows} rows")
        if r.tof is None:
            flags.append("no _tof")
        if any(not e.scorable for e in r.episodes):
            flags.append("unscorable marks")
        rr.append([f"[{r.name}]({plots_rel}/{r.name[:-4]}.png)", r.object, r.trial, f"{r.duration_s:.0f}",
                   f"{r.monitor_s():.0f}", f"{sum(x[1] for x in s.slips)}/{len(s.slips)}",
                   f"{sum(s.dists)}/{len(s.dists)}", len(s.fa_times), s.n_events, ", ".join(flags)])
    L += [table(["recording", "object", "trial", "s", "MONITOR s", "slips det.", "dist. with event", "quiet FA",
                 "events", "notes"], rr), ""]
    dw = [(r.name, w) for r in recs for w in r.data_warnings()] + \
         [(n, w) for n, ws in sorted(rwarn.items()) for w in dict.fromkeys(ws)]
    if dw:
        L += ["### Data warnings", "", "From the loader and the replay tool's `warning:` lines.", "",
              table(["recording", "warning"], dw), ""]
    if per_log:
        L += ["### Firmware-logged slip events vs replay", "",
              table(["recording", "settings", "logged", "replayed", f"matched ({LOG_LAG_ROWS[0]}–{LOG_LAG_ROWS[1]} rows lag)"], per_log), ""]
    if uns:
        L += ["### Not scorable episodes", "",
              table(["recording", "mark", "t (s from start)", "why"],
                    [[e.rec, e.type, f"{(e.t_mark - next(r for r in recs if r.name == e.rec).rows['t_ms'].iloc[0]) / 1000:.1f}",
                      e.why_not] for e in uns]), ""]
    L += ["## Files", "", "- `best_params.json` — chosen parameters, metrics, per object/type, leave-one-object-out.",
          "- `config_patch.txt` — `slipXxx=value` lines for the GripConfig fields (see README for how to apply).",
          "- `results_all_sets.csv` — every evaluated parameter set with its overall metrics.",
          "- `plots/` — per-recording and per-object PNGs.", ""]
    with open(os.path.join(out, "report.md"), "w") as fh:
        fh.write("\n".join(L))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--replay", default="auto", help="auto | python | command line honouring the slip_replay contract")
    ap.add_argument("--notes", default=None, help="notes file (default: auto-detect file,object,... in --data)")
    ap.add_argument("--max-fa-per-min", type=float, default=0.5)
    ap.add_argument("--max-dist-frac", type=float, default=0.30)
    ap.add_argument("--latency-tie-ms", type=float, default=se.LAT_TIE_MS,
                    help="mean latencies within the same bin of this width tie (then fewer false alarms wins)")
    ap.add_argument("--tol", type=float, default=sgdata.TOL_S, help="scoring tolerance, s")
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--quick", action="store_true", help="tiny grid (tests)")
    args = ap.parse_args(argv)
    global _RECS, _REPLAY, _TOL
    se.LAT_TIE_MS = args.latency_tie_ms
    t0 = time.time()
    os.makedirs(args.out, exist_ok=True)
    recs = sgdata.load_dataset(args.data, args.notes)
    if not recs:
        sys.exit(f"no recordings in {args.data}")
    replay = se.Replay(args.replay)
    _RECS, _REPLAY, _TOL = recs, replay, args.tol
    if replay.inproc:
        for r in recs:
            r.arrays()
    defaults = replay.defaults()
    grid = QUICK_GRID if args.quick else FULL_GRID
    global _GRID
    _GRID = grid
    lim = (args.max_fa_per_min, args.max_dist_frac)
    print(f"[tune] {len(recs)} recordings, replay: {replay.label}", file=sys.stderr)
    ev = Evaluator(recs, replay, args.workers)
    A, B = stage_sets(defaults, grid)
    ev.evaluate([defaults] + A + B)
    print(f"[tune] stages A+B: {len(ev.cache)} sets in {time.time() - t0:.0f} s", file=sys.stderr)
    idx_all = list(range(len(recs)))
    best, cands = choose(ev, A, B, defaults, idx_all, lim, grid)
    folds = []
    slip_objs = sorted({r.object for r in recs for e in r.episodes if e.kind == "slip" and e.scorable})
    if len(slip_objs) >= 2:
        for o in slip_objs:
            tr = [i for i, r in enumerate(recs) if r.object != o]
            te = [i for i, r in enumerate(recs) if r.object == o]
            b, _ = choose(ev, A, B, defaults, tr, lim, grid)
            folds.append({"object": o, "params": {k: float(v) for k, v in b.items()},
                          "train": ev.agg(b, tr), "held_out": ev.agg(b, te)})
    # final choice over every evaluated set (the folds' fusion candidates included; no leakage for the all-data fit)
    best = ranked(ev, [ev.params[k] for k in sorted(ev.cache)], idx_all, lim)[0]
    runtime = time.time() - t0
    write_outputs(args, recs, ev, defaults, best, folds, A, B, cands, runtime)
    a = ev.agg(best, idx_all)
    print(f"[tune] {len(ev.cache)} sets, {ev.runs} replays, {runtime:.0f} s. chosen: {param_summary(best)}",
          file=sys.stderr)
    print(f"[tune] detection {a['n_det']}/{a['n_slip']} ({100 * a['det_rate']:.0f} %), FA {a['fa_per_min']:.2f}/min, "
          f"disturbances {100 * a['dist_rate']:.0f} %, latency median {a['lat_median_ms']:.0f} ms", file=sys.stderr)
    return best, a


if __name__ == "__main__":
    main()
