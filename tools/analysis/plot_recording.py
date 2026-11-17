#!/usr/bin/env python3
"""One PNG per recording + one overview PNG per object.

    plot_recording.py --data <recordings dir> --out <plots dir> [--params best_params.json] [--replay auto]

Per recording (time axis = s from the first row):
  a  f1..f6 filtered, FSM states shaded, marks / events as vertical lines
  b  raw vs filtered for the pad with the highest force
  c  servo position
  d  width / range / height and rad1..3
  e  replay features: fsr_drop with fsrDropFrac, and tof_zones with tofZoneCount (two panels, one scale each),
     slip events of the replay (black) and of the firmware log (if any), episode windows shaded
Overview per object: one row per recording, both features as a multiple of their threshold (1 = fires).
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import sgdata  # noqa: E402
import slipeval as se  # noqa: E402

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]   # validated categorical order
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
STATE_FILL = {"IDLE": None, "START": "#f0efec", "PRE_TOUCH": "#e7eef8", "GRASP": "#fde9df", "MONITOR": "#e3f4ec",
              "OPEN": "#f6efd9", "FAULT": "#f8dada"}
MARK_STYLE = {"pull": ("#d03b3b", "-"), "twist": ("#d03b3b", "-"), "slide": ("#d03b3b", "-"),
              "other": ("#ec835a", "--"), "end": (INK2, ":")}
SLIP_WIN, DIST_WIN = "#f6c9c9", "#fbe2c8"

plt.rcParams.update({"font.size": 8, "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
                     "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 1.2,
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})


def _states(rec, t0):
    st = rec.rows["state"].to_numpy()
    t = (rec.rows["t_ms"].to_numpy() - t0) / 1000
    out, i = [], 0
    while i < len(st):
        j = i
        while j + 1 < len(st) and st[j + 1] == st[i]:
            j += 1
        out.append((st[i], t[i], t[j] + 0.01))
        i = j + 1
    return out


def _shade_states(ax, spans, label=False):
    for s, a, b in spans:
        c = STATE_FILL.get(s)
        if c:
            ax.axvspan(a, b, color=c, lw=0, zorder=0)
            if label and b - a > 0.6:
                ax.text((a + b) / 2, 1.0, s, transform=ax.get_xaxis_transform(), ha="center", va="bottom",
                        fontsize=7, color=INK2)


def _shade_episodes(ax, rec, t0):
    for e in rec.episodes:
        lo, hi = e.window()
        ax.axvspan((lo - t0) / 1000, (hi - t0) / 1000, color=SLIP_WIN if e.kind == "slip" else DIST_WIN, lw=0,
                   zorder=0, alpha=0.8 if e.scorable else 0.35)


def _marks(ax, rec, t0, label=False):
    for _, tm, typ in rec.marks():
        c, ls = MARK_STYLE.get(typ, (INK2, ":"))
        x = (tm - t0) / 1000
        ax.axvline(x, color=c, ls=ls, lw=1.0, zorder=3)
        if label:
            ax.text(x, 1.0, typ, transform=ax.get_xaxis_transform(), rotation=90, ha="right", va="top",
                    fontsize=7, color=c)


def _events(ax, t_ev, color=INK, ls="-", lw=1.4):
    for x in t_ev:
        ax.axvline(x, color=color, ls=ls, lw=lw, zorder=4)


def plot_one(rec, out, params, path):
    t0 = int(rec.rows["t_ms"].iloc[0])
    t = (rec.rows["t_ms"].to_numpy() - t0) / 1000
    spans = _states(rec, t0)
    num = lambda c: rec.rows[c].to_numpy(dtype=float)  # noqa: E731
    fig, axs = plt.subplots(6, 1, figsize=(12, 13), sharex=True,
                            gridspec_kw={"height_ratios": [3, 1.6, 1.2, 1.6, 1.4, 1.4]})
    title = f"{rec.name}  —  object {rec.object}, trial {rec.trial}" + (f", {rec.notes}" if rec.notes else "")
    if rec.synthetic:
        title += "   [SYNTHETIC]"
    fig.suptitle(title, x=0.01, ha="left", fontsize=10, color=INK)
    # a
    ax = axs[0]
    _shade_states(ax, spans, label=True)
    for i in range(6):
        ax.plot(t, num(f"f{i + 1}"), color=SERIES[i], label=f"FSR{i + 1}")
    _marks(ax, rec, t0, label=True)
    other = rec.rows[rec.rows["event"].str.match(r"^(contact:|grasp_reached|tighten)")]
    for x in (other["t_ms"].to_numpy() - t0) / 1000:
        ax.axvline(x, color=GRID, lw=0.8, zorder=1)
    logged = (rec.rows.loc[rec.rows["event"].str.startswith("slip:"), "t_ms"].to_numpy() - t0) / 1000
    _events(ax, logged, color="#6250d6", ls="--", lw=1.2)
    ax.set_ylabel("filtered force (N)")
    ax.legend(ncol=6, loc="upper left", fontsize=7, frameon=False)
    # b
    ax = axs[1]
    _shade_states(ax, spans)
    fmax = [np.nanmax(num(f"f{i + 1}")) if np.isfinite(num(f"f{i + 1}")).any() else -1 for i in range(6)]
    k = int(np.argmax(fmax))
    ax.plot(t, num(f"r{k + 1}"), color=SERIES[k], alpha=0.35, lw=0.8, label=f"raw r{k + 1}")
    ax.plot(t, num(f"f{k + 1}"), color=SERIES[k], label=f"filtered f{k + 1}")
    ax.set_ylabel(f"FSR{k + 1} (N)")
    ax.legend(loc="upper left", fontsize=7, frameon=False)
    # c
    ax = axs[2]
    _shade_states(ax, spans)
    ax.plot(t, num("pos"), color=SERIES[0])
    ax.set_ylabel("position (counts)")
    # d
    ax = axs[3]
    _shade_states(ax, spans)
    for i, c in enumerate(["width_mm", "range_mm", "height_mm", "rad1", "rad2", "rad3"]):
        ax.plot(t, num(c), color=SERIES[i], label=c.replace("_mm", ""), lw=1.0 if i < 3 else 0.8,
                ls="-" if i < 3 else "--")
    ax.set_ylabel("object (mm)")
    ax.legend(ncol=6, loc="upper left", fontsize=7, frameon=False)
    # e
    ev_t = (out["t"][out["slip"]] - t0) / 1000
    ax = axs[4]
    _shade_episodes(ax, rec, t0)
    ax.plot(t, out["fsr_drop"], color=SERIES[1], label="fsr_drop")
    ax.axhline(params["fsrDropFrac"], color=INK2, ls="--", lw=0.9, label=f"fsrDropFrac {params['fsrDropFrac']:g}")
    ax.fill_between(t, 0, 1, where=~out["active"], transform=ax.get_xaxis_transform(), color="#f0efec", lw=0, zorder=0)
    _marks(ax, rec, t0)
    _events(ax, ev_t)
    ax.set_ylabel("FSR drop (×baseline)")
    ax.set_ylim(0, max(1.05, float(np.nanmax(out["fsr_drop"])) * 1.05 if len(t) else 1.05))
    ax.legend(loc="upper left", fontsize=7, frameon=False)
    ax = axs[5]
    _shade_episodes(ax, rec, t0)
    ax.step(t, out["tof_zones"], where="post", color=SERIES[2], label="tof_zones")
    ax.axhline(params["tofZoneCount"], color=INK2, ls="--", lw=0.9, label=f"tofZoneCount {params['tofZoneCount']:g}")
    ax.fill_between(t, 0, 1, where=~out["active"], transform=ax.get_xaxis_transform(), color="#f0efec", lw=0, zorder=0)
    _marks(ax, rec, t0)
    _events(ax, ev_t)
    ax.set_ylabel("ToF zones changed")
    ax.set_xlabel("time from recording start (s)   ·   black lines = replay slip events, purple dashed = logged "
                  "slip events, red/orange = slip/disturbance marks and windows, grey band = detector not armed")
    if rec.tof is None:
        ax.text(0.5, 0.5, "no _tof file", transform=ax.transAxes, ha="center", color=INK2)
    ax.legend(loc="upper left", fontsize=7, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(path, dpi=90)
    plt.close(fig)


def plot_overview(obj, items, params, path):
    n = len(items)
    fig, axs = plt.subplots(n, 1, figsize=(12, 1.25 * n + 0.8), squeeze=False)
    fig.suptitle(f"{obj}: features as a multiple of their threshold (1 = flag, clipped at 3); black = replay slip events"
                 + ("   [SYNTHETIC]" if any(r.synthetic for r, _ in items) else ""), x=0.01, ha="left", fontsize=10)
    for ax, (rec, out) in zip(axs[:, 0], items):
        t0 = int(rec.rows["t_ms"].iloc[0])
        t = (out["t"] - t0) / 1000
        _shade_episodes(ax, rec, t0)
        ax.plot(t, np.minimum(out["fsr_drop"] / max(params["fsrDropFrac"], 1e-6), 2.97), color=SERIES[1], lw=0.9, label="fsr_drop")
        ax.step(t, np.minimum(out["tof_zones"] / max(params["tofZoneCount"], 1), 2.97), where="post", color=SERIES[2], lw=0.9,
                label="tof_zones")
        ax.axhline(1, color=INK2, ls="--", lw=0.7)
        _events(ax, (t[out["slip"]]), lw=1.2)
        _marks(ax, rec, t0)
        ax.set_ylim(0, 3)
        ax.set_ylabel(rec.trial[:14], fontsize=7)
        ax.text(0.995, 0.92, rec.name, transform=ax.transAxes, ha="right", va="top", fontsize=7, color=INK2)
    axs[0, 0].legend(loc="upper left", fontsize=7, frameon=False, ncol=2)
    axs[-1, 0].set_xlabel("time from recording start (s)")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(path, dpi=80)
    plt.close(fig)


_G = {}


def _job(i):
    rec = _G["recs"][i]
    out = _G["replay"].run(rec, _G["params"])
    plot_one(rec, out, _G["params"], os.path.join(_G["out"], rec.name[:-4] + ".png"))
    return i, {k: out[k] for k in ("t", "fsr_drop", "tof_zones", "slip", "active")}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--params", default=None, help="best_params.json (default: replay defaults)")
    ap.add_argument("--replay", default="auto")
    ap.add_argument("--notes", default=None)
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 1)
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    recs = sgdata.load_dataset(a.data, a.notes)
    replay = se.Replay(a.replay)
    params = replay.defaults()
    if a.params:
        with open(a.params) as fh:
            params.update({k: float(v) for k, v in json.load(fh)["params"].items()})
    _G.update(recs=recs, replay=replay, params=params, out=a.out)
    if a.workers > 1:
        with mp.get_context("fork").Pool(a.workers) as pool:
            res = dict(pool.imap(_job, range(len(recs))))
    else:
        res = dict(map(_job, range(len(recs))))
    for obj in sorted({r.object for r in recs}):
        items = [(r, res[i]) for i, r in enumerate(recs) if r.object == obj]
        plot_overview(obj, items, params, os.path.join(a.out, f"overview_{obj}.png"))
    print(f"[plot] {len(recs)} recordings + {len({r.object for r in recs})} overviews -> {a.out} "
          f"(replay: {replay.label})", file=sys.stderr)


if __name__ == "__main__":
    main()
