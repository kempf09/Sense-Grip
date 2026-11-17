"""Replay runner (C++ tool or Python stand-in) and episode scoring shared by tune_slip.py and plot_recording.py."""
from __future__ import annotations

import io
import math
import os
import shlex
import subprocess
import sys
from dataclasses import dataclass

import numpy as np
import pandas as pd

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay_py  # noqa: E402
import sgdata  # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
CPP_TOOL = os.path.join(REPO, "tools", "slip_replay", "slip_replay")
OUT_HEADER = replay_py.OUT_HEADER.split(",")
PARAM_NAMES = replay_py.PARAM_NAMES


# ---------------------------------------------------------------------------------------------------------------
class Replay:
    """`spec`: 'auto' (C++ binary if built, else the Python stand-in in-process), 'python' (stand-in in-process),
    or any command line that honours the slip_replay contract (e.g. 'python3 tools/analysis/replay_py.py')."""

    def __init__(self, spec: str = "auto"):
        spec = (spec or "auto").strip()
        if spec == "auto":
            spec = CPP_TOOL if os.access(CPP_TOOL, os.X_OK) else "python"
        self.spec = spec
        self.inproc = spec == "python"
        self.cmd = None if self.inproc else shlex.split(spec)
        if self.inproc:
            self.label = "Python stand-in tools/analysis/replay_py.py (in-process)"
        elif os.path.abspath(self.cmd[0]) == CPP_TOOL:
            self.label = f"C++ replay tool {os.path.relpath(CPP_TOOL, REPO)} (links SenseGrip/slip_core.cpp)"
        else:
            self.label = f"command: {spec}"

    def defaults(self) -> dict:
        if self.inproc:
            return {k: float(f"{float(v):g}") for k, v in replay_py.coerce({}).items()}   # as printed
        out = subprocess.run(self.cmd + ["--print-defaults"], capture_output=True, text=True, check=True).stdout
        d = {}
        for line in out.splitlines():
            if "=" in line:
                k, v = line.strip().split("=", 1)
                d[k] = float(v)
        missing = [k for k in PARAM_NAMES if k not in d]
        if missing:
            raise RuntimeError(f"replay --print-defaults lacks {missing}")
        return d

    def run(self, rec: sgdata.Recording, params: dict, force_active: bool = False) -> dict:
        """Per-row outputs as numpy arrays: active, fsr_drop, fsr_flag, tof_zones, tof_disp, tof_flag, slip, reason."""
        if self.inproc:
            arr = rec.arrays()
            r = replay_py.run(arr, params, force_active)
            r["reason"] = np.array([replay_py.REASONS[x] if s else "" for x, s in zip(r["reason"], r["slip"])], object)
            r["t"] = arr["t"]
            return r
        args = list(self.cmd)
        for k in PARAM_NAMES:
            if k in params:
                args += ["--param", f"{k}={params[k]:g}"]
        if force_active:
            args.append("--force-active")
        args += ["--rows", rec.path]
        if rec.tof_path:
            args += ["--tof", rec.tof_path]
        p = subprocess.run(args, capture_output=True, text=True)
        if p.returncode != 0:
            raise RuntimeError(f"replay failed on {rec.name}: {p.stderr.strip()[:300]}")
        out = parse_replay_csv(p.stdout, rec)
        out["warnings"] = [ln.strip().replace(rec.path, rec.name).replace(rec.tof_path or "\0", rec.name[:-4] + "_tof.csv")
                           for ln in p.stderr.splitlines() if ln.startswith("warning:")]
        return out


def parse_replay_csv(text: str, rec: sgdata.Recording | None = None) -> dict:
    first = text.split("\n", 1)[0].strip().split(",")
    if first != OUT_HEADER:
        raise RuntimeError(f"replay output header mismatch: {first}")
    df = pd.read_csv(io.StringIO(text), dtype={"reason": str, "state": str}, keep_default_na=False,
                     na_values={"tof_disp_mm": [""]})
    if rec is not None and len(df) != len(rec.rows):
        raise RuntimeError(f"replay output has {len(df)} rows, recording {rec.name} has {len(rec.rows)}")
    return {"t": df["t_ms"].to_numpy(np.int64), "active": df["active"].to_numpy() == 1,
            "fsr_drop": df["fsr_drop"].to_numpy(np.float32), "fsr_flag": df["fsr_flag"].to_numpy() == 1,
            "tof_zones": df["tof_zones"].to_numpy(np.int64), "tof_disp": df["tof_disp_mm"].to_numpy(np.float32),
            "tof_flag": df["tof_flag"].to_numpy() == 1, "slip": df["slip"].to_numpy() == 1,
            "reason": df["reason"].fillna("").to_numpy(object)}


# ---------------------------------------------------------------------------------------------------------------
@dataclass
class RecScore:
    name: str
    obj: str
    slips: list            # (type, detected bool, latency_ms or None, t_mark)
    dists: list            # triggered bool
    quiet_s: float
    fa_times: list         # quiet false-alarm event times (ms)
    post_slip_fa: int      # quiet false alarms within 10 s after a slip window
    n_events: int
    unscorable_events: int


def score(rec: sgdata.Recording, out: dict, tol_s: float = sgdata.TOL_S) -> RecScore:
    t = out["t"]
    ev_t = t[out["slip"]]
    eps = rec.episodes
    wins = [e.window(tol_s) for e in eps]
    in_any = np.zeros(len(t), bool)
    for lo, hi in wins:
        in_any |= (t >= lo) & (t <= hi)
    slips, dists, unsc = [], [], 0
    for e, (lo, hi) in zip(eps, wins):
        hit = ev_t[(ev_t >= lo) & (ev_t <= hi)]
        if not e.scorable:
            unsc += len(hit)
            continue
        if e.kind == "slip":
            slips.append((e.type, bool(len(hit)), int(hit[0] - e.t_mark) if len(hit) else None, e.t_mark))
        else:
            dists.append(bool(len(hit)))
    dtm = np.minimum(np.diff(t, append=t[-1] + 10 if len(t) else 0), 20) if len(t) else np.zeros(0)
    quiet = out["active"] & ~in_any
    quiet_s = float(dtm[quiet].sum()) / 1000.0
    fa_mask = out["slip"] & quiet
    fa_t = t[fa_mask].tolist()
    slip_wins = [w for e, w in zip(eps, wins) if e.kind == "slip"]
    post = sum(1 for ft in fa_t if any(hi < ft <= hi + 10000 for _, hi in slip_wins))
    return RecScore(rec.name, rec.object, slips, dists, quiet_s, fa_t, post, int(out["slip"].sum()), unsc)


def aggregate(scores: list[RecScore]) -> dict:
    sl = [s for r in scores for s in r.slips]
    ds = [d for r in scores for d in r.dists]
    lat = [s[2] for s in sl if s[1]]
    q = sum(r.quiet_s for r in scores)
    nfa = sum(len(r.fa_times) for r in scores)
    a = {"n_slip": len(sl), "n_det": sum(1 for s in sl if s[1]), "n_dist": len(ds), "n_dist_trig": sum(ds),
         "quiet_min": q / 60.0, "n_fa": nfa, "post_slip_fa": sum(r.post_slip_fa for r in scores),
         "lat_mean_ms": float(np.mean(lat)) if lat else float("nan"),
         "lat_median_ms": float(np.median(lat)) if lat else float("nan"),
         "lat_p90_ms": float(np.percentile(lat, 90)) if lat else float("nan")}
    a["det_rate"] = a["n_det"] / a["n_slip"] if a["n_slip"] else float("nan")
    a["fa_per_min"] = nfa / (q / 60.0) if q > 0 else (0.0 if nfa == 0 else float("inf"))
    a["dist_rate"] = a["n_dist_trig"] / a["n_dist"] if a["n_dist"] else 0.0
    return a


def by_slip_type(scores: list[RecScore]) -> dict:
    out = {}
    for typ in sgdata.SLIP_TYPES:
        sl = [s for r in scores for s in r.slips if s[0] == typ]
        if sl:
            lat = [s[2] for s in sl if s[1]]
            out[typ] = {"n_slip": len(sl), "n_det": sum(1 for s in sl if s[1]),
                        "det_rate": sum(1 for s in sl if s[1]) / len(sl),
                        "lat_median_ms": float(np.median(lat)) if lat else float("nan")}
    return out


def feasible(a: dict, max_fa: float, max_dist: float) -> bool:
    return a["fa_per_min"] <= max_fa + 1e-12 and a["dist_rate"] <= max_dist + 1e-12


LAT_TIE_MS = 20.0     # mean latencies closer than this (2 rows) count as equal -> fewer false alarms decides


def rank_key(a: dict, max_fa: float, max_dist: float):
    """Sort ascending = better. Feasible first; max detection; then lower mean latency (in LAT_TIE_MS bins); then
    fewer false alarms, fewer disturbance events. Infeasible sets rank by how far they exceed the limits."""
    lat = a["lat_mean_ms"] if a["lat_mean_ms"] == a["lat_mean_ms"] else 1e9
    lat = math.floor(lat / LAT_TIE_MS) if LAT_TIE_MS > 0 else lat
    det = a["det_rate"] if a["det_rate"] == a["det_rate"] else 0.0
    if feasible(a, max_fa, max_dist):
        return (0, 0.0, -round(det, 9), lat, a["fa_per_min"], a["dist_rate"])
    excess = max(0.0, a["fa_per_min"] - max_fa) / max(max_fa, 1e-9) + max(0.0, a["dist_rate"] - max_dist) / max(max_dist, 1e-9)
    return (1, round(excess, 9), -round(det, 9), lat, a["fa_per_min"], a["dist_rate"])
