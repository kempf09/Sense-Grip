#!/usr/bin/env python3
"""Pure-Python stand-in for tools/slip_replay (same command-line contract, same algorithm).

    replay_py.py [--param name=value ...] --rows <name>.csv [--tof <name>_tof.csv] > out.csv
    replay_py.py --print-defaults
    replay_py.py --force-active ...      (detector on every row: manual recordings never reach MONITOR)

Reference: SenseGrip/slip_core.cpp (slipStep). This file mirrors it step by step, vectorised per MONITOR segment, in
float32 like the firmware. Mirrored details (also the contract the C++ tool is cross-checked against, see
test_pipeline.py):
  * input values exactly as logged (rounded: forces = int mN / 1000, mm = int 0.1 mm / 10, in float32), as the
    firmware's slipInputFromRow builds them.
  * active input = state == MONITOR; any other state resets the detector. --force-active: active from the first row
    where any pad force >= fsrMinForceN (latched) to the end of the file, whatever the state.  Output `active` = baseline taken
    (MONITOR for >= settleMs, measured from the first MONITOR row's t_ms).
  * baseline at the first armed row: per pad filtered force, the latest frame set fed so far (if none, the first frame
    set fed after settle becomes the baseline and is not itself compared), obj_valid/range/height/rad1..3.
  * FSR: per pad with baseline >= fsrMinForceN: (max over the force history within min(fsrWindowMs, 600) ms, at most 64
    rows, since MONITOR entry - current) / baseline, a NaN current force counting as 0 (the maximum starts from it).
    fsr_flag when drop >= fsrDropFrac continuously for >= fsrConfirmMs (30 ms = 4 rows at 100 Hz).
  * ToF: frame records are fed at main row seq S = every record with row_seq <= S (several per row -> only the latest
    is compared; the firmware logs one record per single-sensor update, each holding all 3 sensors' latest zones).
    Zone counted when valid (1..3999) and <= tofNearMm at baseline, sensor online at baseline and now, and now
    invalid or |now - base| > max(tofZoneMm, tofZoneFrac * base). tof_flag when the count has been >= tofZoneCount
    continuously for >= tofConfirmMs: from the row time at which the first record of the run was fed to the current
    row time (re-evaluated on every row, the count only changes on rows that feed a record).
  * displacement = max |delta| of range, height, rad1..3 (NaN-ignoring) while obj_valid now and at baseline; it raises
    tof_flag only when tofDispMm > 0 and displacement >= max(tofDispMm, 0.1 * baseline range).
  * no --tof: no frames, tof_zones stays 0 (displacement still works).
  * fusion: mode 0 fsr, 1 tof, 3 both flagged within fuseWindowMs (each flag's last time), anything else = either.
    Event on a candidate row unless inside holdoffMs of the previous event. reason (printed only on event rows) =
    the row's flags; mode 3 always fsr+tof.
  * re-baseline after an event: at the first row with t >= event t + holdoffMs the baseline is taken again and the
    force history restarts at that row (forces,
    the latest frame if any, the object measurement); FSR/ToF confirmation runs, the zone count and the "flag seen"
    state for mode 3 restart.
  * inactive rows print `0,0.000,0,0,,0,0,`; fsr_drop %.3f, tof_disp_mm %.1f or empty.
"""
from __future__ import annotations

import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import sgdata  # noqa: E402

PARAM_NAMES = ["fsrDropFrac", "fsrWindowMs", "fsrMinForceN", "fsrConfirmMs", "tofNearMm", "tofZoneMm", "tofZoneFrac",
               "tofZoneCount", "tofConfirmMs", "tofDispMm", "mode", "fuseWindowMs", "settleMs", "holdoffMs"]
DEFAULTS = {"fsrDropFrac": 0.30, "fsrWindowMs": 300, "fsrMinForceN": 0.15, "fsrConfirmMs": 30, "tofNearMm": 350,
            "tofZoneMm": 12, "tofZoneFrac": 0.10, "tofZoneCount": 4, "tofConfirmMs": 150, "tofDispMm": 0.0,
            "mode": 2, "fuseWindowMs": 500, "settleMs": 500, "holdoffMs": 1500}
_FLOAT = {"fsrDropFrac", "fsrMinForceN", "tofZoneFrac", "tofDispMm"}
_U8 = {"tofZoneCount", "mode"}
SLIP_HIST = 64
OUT_HEADER = "seq,t_ms,state,active,fsr_drop,fsr_flag,tof_zones,tof_disp_mm,tof_flag,slip,reason"
REASONS = ["", "fsr", "tof", "fsr+tof"]


def coerce(params: dict | None) -> dict:
    """Full parameter set with the C++ storage types applied (float32, u16/u8 rounded and clamped)."""
    p = dict(DEFAULTS)
    for k, v in (params or {}).items():
        if k not in DEFAULTS:
            raise KeyError(f"unknown parameter {k}")
        p[k] = float(v)
    out = {}
    for k, v in p.items():
        v = float(np.float32(v))
        if k in _FLOAT:
            out[k] = np.float32(v)
        else:
            hi = 255 if k in _U8 else 65535
            out[k] = 0 if v < 0 else hi if v > hi else int(np.float32(v) + np.float32(0.5))
    return out


def _segments(mask: np.ndarray):
    d = np.diff(np.concatenate([[0], mask.astype(np.int8), [0]]))
    return zip(np.nonzero(d == 1)[0], np.nonzero(d == -1)[0])


def _ffill_time(flag: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Time of the latest True at or before each index (-1 when none)."""
    idx = np.where(flag, np.arange(len(flag)), -1)
    idx = np.maximum.accumulate(idx) if len(idx) else idx
    return np.where(idx >= 0, t[np.maximum(idx, 0)], -1)


def _epoch(arr, P, a, k, s, tr_all, mx, cur0, bf, pending):
    """Features from armed row s (segment-relative to k) to the segment end with the baseline taken at row s.
    bf/pending: baseline frame index, or pending = the first frame record fed after s becomes the baseline."""
    frames, fvalid = arr["frames"], arr["fvalid"]
    g = a + k + s                                   # absolute row of the baseline
    tr = tr_all[s:]
    nr = len(tr)
    # FSR
    base = arr["force"][g].copy()
    with np.errstate(divide="ignore", invalid="ignore"):
        d = (mx[s:] - cur0[s:]) / base
    elig = base >= P["fsrMinForceN"]
    d = np.where(elig[None, :] & (d > 0), d, np.float32(0)).astype(np.float32)
    drop = d.max(axis=1) if d.shape[1] else np.zeros(nr, np.float32)
    over = drop >= P["fsrDropFrac"]
    start = over & ~np.concatenate([[False], over[:-1]])
    since = _ffill_time(start, tr)
    fsr = over & ((tr - since) >= P["fsrConfirmMs"])
    # ToF zones
    L = arr["last_frame"][g:g + nr]
    N = arr["frame_new"][g:g + nr]
    zones = np.zeros(nr, np.int64)
    zflag = np.zeros(nr, bool)
    ev_rows = np.zeros(0, np.int64)
    if pending:
        nz = np.nonzero(N)[0]
        if len(nz):
            bf, pending = int(L[nz[0]]), False
            ev_rows = nz[1:]
    elif bf >= 0:
        ev_rows = np.nonzero(N)[0]
    if bf >= 0 and len(ev_rows):
        b = frames[bf].astype(np.float32)
        bok = (frames[bf] > 0) & (frames[bf] < 4000) & (frames[bf] <= P["tofNearMm"])
        bon = np.array([(fvalid[bf] >> q) & 1 for q in range(3)], bool)
        lim = np.maximum(np.float32(P["tofZoneMm"]), P["tofZoneFrac"] * b)
        fi = L[ev_rows]
        V = frames[fi]
        von = np.stack([(fvalid[fi] >> q) & 1 for q in range(3)], axis=1).astype(bool)
        changed = (~((V > 0) & (V < 4000))) | (np.abs(V.astype(np.float32) - b[None]) > lim[None])
        cnt = (changed & bok[None] & (bon[None, :, None] & von[:, :, None])).sum(axis=(1, 2))
        hit = cnt >= P["tofZoneCount"]
        hstart = hit & ~np.concatenate([[False], hit[:-1]])
        hsince = _ffill_time(hstart, tr[ev_rows])
        pos = np.searchsorted(ev_rows, np.arange(nr), side="right") - 1
        have = pos >= 0
        zones[have] = cnt[pos[have]]
        tover = np.zeros(nr, bool)
        tover[have] = hit[pos[have]]
        tsince = np.where(have, hsince[np.maximum(pos, 0)], 0)
        zflag = tover & ((tr - tsince) >= P["tofConfirmMs"])
    # displacement
    disp = np.full(nr, np.nan, np.float32)
    dflag = np.zeros(nr, bool)
    if arr["obj"][g]:
        r0, h0, rad0 = arr["range"][g], arr["height"][g], arr["rad"][g]
        ob = arr["obj"][g:g + nr]
        with np.errstate(invalid="ignore"):
            dd = np.abs(arr["range"][g:g + nr] - r0)
            dd = np.fmax(dd, np.abs(arr["height"][g:g + nr] - h0))
            for q in range(3):
                rs = arr["rad"][g:g + nr, q]
                both = ~np.isnan(rs) & ~np.isnan(rad0[q])
                dd = np.where(both, np.fmax(dd, np.abs(rs - rad0[q])), dd)
        disp = np.where(ob, dd, np.float32(np.nan)).astype(np.float32)
        if P["tofDispMm"] > 0:
            thr = np.fmax(P["tofDispMm"], np.float32(0.1) * r0)
            with np.errstate(invalid="ignore"):
                dflag = ob & (disp >= thr)
    tof = zflag | dflag
    # fusion
    mode = P["mode"]
    if mode == 0:
        cand = fsr.copy()
    elif mode == 1:
        cand = tof.copy()
    elif mode == 3:
        lf, lt = _ffill_time(fsr, tr), _ffill_time(tof, tr)
        cand = (fsr | tof) & (lf >= 0) & (lt >= 0) & ((tr - lf) <= P["fuseWindowMs"]) & ((tr - lt) <= P["fuseWindowMs"])
    else:
        cand = fsr | tof
    reason = fsr.astype(np.int8) | (tof.astype(np.int8) << 1)
    if mode == 3:
        reason = np.where(cand, 3, reason).astype(np.int8)
    return {"drop": drop, "fsr": fsr, "zones": zones, "disp": disp, "tof": tof, "cand": cand,
            "reason": reason}, bf, pending


def run(arr: dict, params: dict | None = None, force_active: bool = False) -> dict:
    """Detector over one recording's arrays (sgdata.detector_arrays). Returns per-row output arrays.
    force_active: slip_replay --force-active (manual recordings): active from the first row where any pad force
    >= fsrMinForceN (latched) to the end, whatever the state."""
    P = coerce(params)
    n = len(arr["t"])
    out = {"active": np.zeros(n, bool), "fsr_drop": np.zeros(n, np.float32), "fsr_flag": np.zeros(n, bool),
           "tof_zones": np.zeros(n, np.int64), "tof_disp": np.full(n, np.nan, np.float32),
           "tof_flag": np.zeros(n, bool), "slip": np.zeros(n, bool), "reason": np.zeros(n, np.int8)}
    win = min(P["fsrWindowMs"], 600)
    if force_active:
        active_in = np.zeros(n, bool)
        with np.errstate(invalid="ignore"):
            touch = np.nonzero((arr["force"] >= P["fsrMinForceN"]).any(axis=1))[0]
        if len(touch):
            active_in[touch[0]:] = True
    else:
        active_in = arr["monitor"]
    for a, e in _segments(active_in):
        t = arr["t"][a:e]
        armed = np.nonzero((t - t[0]) >= P["settleMs"])[0]
        if not len(armed):
            continue
        k = int(armed[0])
        m = e - a
        R = np.arange(k, m)
        tr = t[k:]
        F = arr["force"][a:e]
        # window maximum per pad (independent of the baseline; history since the segment start, <= 64 rows)
        cur0 = np.where(np.isnan(F[k:]), np.float32(0), F[k:])     # NaN current counts as 0

        def wmax(lo):
            # window maximum using history rows j >= lo only (lo = segment start, or the
            # re-baseline row: slip_core keeps only the current step's entry there, histN = 1)
            m_ = cur0.copy()
            for off in range(1, SLIP_HIST):
                j = R - off
                ok = j >= lo
                if not ok.any():
                    break
                ok &= (tr - t[np.maximum(j, 0)]) <= win
                if not ok.any():
                    break
                v = F[np.maximum(j, 0)]
                m_ = np.where(ok[:, None] & (v > m_), v, m_)
            return m_
        mx = wmax(0)
        L0 = arr["last_frame"][a + k]
        bf, pending = (int(L0), False) if L0 >= 0 else (-1, True)
        s, hold_until = 0, None
        while s < len(R):
            f, bf, pending = _epoch(arr, P, a, k, s, tr, mx, cur0, bf, pending)
            ts = tr[s:]
            ok = f["cand"] if hold_until is None else f["cand"] & (ts >= hold_until)
            ci = np.nonzero(ok)[0]
            if len(ci):
                r = int(ci[0])
                hold_until = int(ts[r]) + P["holdoffMs"]
                nxt = max(int(np.searchsorted(ts, hold_until, side="left")), r + 1)   # re-baseline row
            else:
                r, nxt = -1, len(ts)
            sl = slice(a + k + s, a + k + s + nxt)
            out["active"][sl] = True
            out["fsr_drop"][sl] = f["drop"][:nxt]
            out["fsr_flag"][sl] = f["fsr"][:nxt]
            out["tof_zones"][sl] = f["zones"][:nxt]
            out["tof_disp"][sl] = f["disp"][:nxt]
            out["tof_flag"][sl] = f["tof"][:nxt]
            if r >= 0:
                out["slip"][a + k + s + r] = True
                out["reason"][a + k + s + r] = f["reason"][r]
            s += nxt
            if r >= 0 and s < len(R):                 # re-baseline: frame only if one exists at that row
                mx = wmax(k + s)                      # force history restarts at the re-baseline row
                Ls = arr["last_frame"][a + k + s]
                if Ls >= 0:
                    bf, pending = int(Ls), False
    return out


def format_csv(arr: dict, res: dict) -> str:
    lines = [OUT_HEADER]
    seq, t, st = arr["seq"], arr["t"], arr["state"]
    for i in range(len(seq)):
        if res["active"][i]:
            dv = res["tof_disp"][i]
            ds = "" if np.isnan(dv) else f"{float(dv):.1f}"
            lines.append(f"{seq[i]},{t[i]},{st[i]},1,{float(res['fsr_drop'][i]):.3f},{int(res['fsr_flag'][i])},"
                         f"{int(res['tof_zones'][i])},{ds},{int(res['tof_flag'][i])},{int(res['slip'][i])},"
                         f"{REASONS[res['reason'][i]] if res['slip'][i] else ''}")
        else:
            lines.append(f"{seq[i]},{t[i]},{st[i]},0,0.000,0,0,,0,0,")
    return "\n".join(lines) + "\n"


def main(argv: list[str]) -> int:
    params, rows, tof, force = {}, None, None, False
    it = iter(argv)
    for a in it:
        if a == "--print-defaults":
            P = coerce({})
            for k in PARAM_NAMES:
                print(f"{k}={float(P[k]):g}")
            return 0
        if a == "--param":
            kv = next(it, "")
            if "=" not in kv:
                print(f"bad --param {kv!r}", file=sys.stderr)
                return 2
            k, v = kv.split("=", 1)
            if k not in DEFAULTS:
                print(f"unknown parameter {k}", file=sys.stderr)
                return 2
            params[k] = float(v)
        elif a == "--force-active":
            force = True
        elif a == "--rows":
            rows = next(it, None)
        elif a == "--tof":
            tof = next(it, None)
        else:
            print(f"unknown argument {a}", file=sys.stderr)
            return 2
    if not rows:
        print("usage: replay_py.py [--param name=value ...] --rows <name>.csv [--tof <name>_tof.csv]", file=sys.stderr)
        return 2
    rec = sgdata.load_recording(rows)
    if tof is None:            # contract: frames only with --tof
        rec.tof, rec.tof_path = None, None
    elif os.path.abspath(tof) != os.path.abspath(rec.tof_path or ""):
        _, rec.tof = sgdata._read_csv_skip_comments(tof, None)
    arr = rec.arrays()
    sys.stdout.write(format_csv(arr, run(arr, params, force)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
