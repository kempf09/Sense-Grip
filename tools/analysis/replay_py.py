#!/usr/bin/env python3
"""Pure-Python stand-in for tools/slip_replay (same command-line contract, same algorithm).

    replay_py.py [--param name=value ...] --rows <name>.csv [--tof <name>_tof.csv] > out.csv
    replay_py.py --print-defaults

Reference: SenseGrip/slip_core.cpp (slipStep). This file mirrors it step by step, vectorised per MONITOR segment, in
float32 like the firmware. Mirrored details (also the contract the C++ tool is cross-checked against, see
test_pipeline.py):
  * active input = state == MONITOR; any other state resets the detector.  Output `active` = baseline taken
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
    the row's flags; mode 3 always fsr+tof.  No re-baseline after an event (spec §4 as written).
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


def run(arr: dict, params: dict | None = None) -> dict:
    """Detector over one recording's arrays (sgdata.detector_arrays). Returns per-row output arrays."""
    P = coerce(params)
    n = len(arr["t"])
    out = {"active": np.zeros(n, bool), "fsr_drop": np.zeros(n, np.float32), "fsr_flag": np.zeros(n, bool),
           "tof_zones": np.zeros(n, np.int64), "tof_disp": np.full(n, np.nan, np.float32),
           "tof_flag": np.zeros(n, bool), "slip": np.zeros(n, bool), "reason": np.zeros(n, np.int8)}
    win = min(P["fsrWindowMs"], 600)
    frames, fvalid = arr["frames"], arr["fvalid"]
    for a, e in _segments(arr["monitor"]):
        t = arr["t"][a:e]
        rel = t - t[0]
        armed = np.nonzero(rel >= P["settleMs"])[0]
        if not len(armed):
            continue
        k = int(armed[0])
        m = e - a
        R = np.arange(k, m)
        tr = t[k:]
        F = arr["force"][a:e]
        # ---- FSR -------------------------------------------------------------------------------------------
        base = F[k].copy()
        cur0 = np.where(np.isnan(F[k:]), np.float32(0), F[k:])     # NaN current counts as 0
        mx = cur0.copy()
        for off in range(1, SLIP_HIST):
            j = R - off
            ok = j >= 0
            if not ok.any():
                break
            ok &= (tr - t[np.maximum(j, 0)]) <= win
            if not ok.any():
                break
            v = F[np.maximum(j, 0)]
            mx = np.where(ok[:, None] & (v > mx), v, mx)
        with np.errstate(divide="ignore", invalid="ignore"):
            d = (mx - cur0) / base
        elig = base >= P["fsrMinForceN"]
        d = np.where(elig[None, :] & (d > 0), d, np.float32(0)).astype(np.float32)
        drop = d.max(axis=1) if d.shape[1] else np.zeros(len(R), np.float32)
        over = drop >= P["fsrDropFrac"]
        start = over & ~np.concatenate([[False], over[:-1]])
        since = _ffill_time(start, tr)
        fsr = over & ((tr - since) >= P["fsrConfirmMs"])
        # ---- ToF zones ---------------------------------------------------------------------------------------
        L = arr["last_frame"][a + k:e]
        N = arr["frame_new"][a + k:e]
        zones = np.zeros(len(R), np.int64)
        zflag = np.zeros(len(R), bool)
        bf, ev_rows = -1, np.zeros(0, np.int64)
        if L[0] >= 0:
            bf = int(L[0])
            ev_rows = np.nonzero(N)[0]
        else:
            nz = np.nonzero(N)[0]
            if len(nz):
                bf = int(L[nz[0]])
                ev_rows = nz[1:]
        if bf >= 0 and len(ev_rows):
            b = frames[bf].astype(np.float32)
            bok = (frames[bf] > 0) & (frames[bf] < 4000) & (frames[bf] <= P["tofNearMm"])
            bon = np.array([(fvalid[bf] >> s) & 1 for s in range(3)], bool)
            lim = np.maximum(np.float32(P["tofZoneMm"]), P["tofZoneFrac"] * b)
            fi = L[ev_rows]
            V = frames[fi]
            Vf = V.astype(np.float32)
            von = np.stack([(fvalid[fi] >> s) & 1 for s in range(3)], axis=1).astype(bool)
            changed = (~((V > 0) & (V < 4000))) | (np.abs(Vf - b[None]) > lim[None])
            elig_z = bok[None] & (bon[None, :, None] & von[:, :, None])
            cnt = (changed & elig_z).sum(axis=(1, 2))
            hit = cnt >= P["tofZoneCount"]
            # consecutive hits
            # run start (row time of the first frame record of the current run of hits)
            hstart = hit & ~np.concatenate([[False], hit[:-1]])
            hsince = _ffill_time(hstart, tr[ev_rows])
            pos = np.searchsorted(ev_rows, np.arange(len(R)), side="right") - 1
            have = pos >= 0
            zones[have] = cnt[pos[have]]
            over = np.zeros(len(R), bool)
            over[have] = hit[pos[have]]
            since = np.where(have, hsince[np.maximum(pos, 0)], 0)
            zflag = over & ((tr - since) >= P["tofConfirmMs"])
        # ---- displacement ------------------------------------------------------------------------------------
        disp = np.full(len(R), np.nan, np.float32)
        dflag = np.zeros(len(R), bool)
        g = a + k
        if arr["obj"][g]:
            r0, h0, rad0 = arr["range"][g], arr["height"][g], arr["rad"][g]
            ob = arr["obj"][g:e]
            with np.errstate(invalid="ignore"):
                dd = np.abs(arr["range"][g:e] - r0)
                dd = np.fmax(dd, np.abs(arr["height"][g:e] - h0))
                for s in range(3):
                    rs = arr["rad"][g:e, s]
                    both = ~np.isnan(rs) & ~np.isnan(rad0[s])
                    dd = np.where(both, np.fmax(dd, np.abs(rs - rad0[s])), dd)
            disp = np.where(ob, dd, np.float32(np.nan)).astype(np.float32)
            if P["tofDispMm"] > 0:
                thr = np.fmax(P["tofDispMm"], np.float32(0.1) * r0)
                with np.errstate(invalid="ignore"):
                    dflag = ob & (disp >= thr)
        tof = zflag | dflag
        # ---- fusion / events ---------------------------------------------------------------------------------
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
        slip = np.zeros(len(R), bool)
        ci = np.nonzero(cand)[0]
        i = 0
        while i < len(ci):
            r = ci[i]
            slip[r] = True
            until = tr[r] + P["holdoffMs"]
            i = int(np.searchsorted(tr[ci], until, side="left"))
            if i < len(ci) and ci[i] <= r:
                i = int(np.searchsorted(ci, r, side="right"))
        sl = slice(a + k, e)
        out["active"][sl] = True
        out["fsr_drop"][sl] = drop
        out["fsr_flag"][sl] = fsr
        out["tof_zones"][sl] = zones
        out["tof_disp"][sl] = disp
        out["tof_flag"][sl] = tof
        out["slip"][sl] = slip
        out["reason"][sl] = np.where(slip, reason, 0)
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
    params, rows, tof = {}, None, None
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
    sys.stdout.write(format_csv(arr, run(arr, params)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
