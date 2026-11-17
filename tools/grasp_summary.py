#!/usr/bin/env python3
"""Phase 3 check from dashboard recordings: one line per grasp recording.

    python3 tools/grasp_summary.py <recordings_dir> [notes.csv] > grasp_summary.md

For every `grasp_*.csv` (the `_tof.csv` files are ignored) it reports: object (notes file, else
the class in the name), result (`# started:` comment), class used, time in START / PRE_TOUCH
approach / PRE_TOUCH slow / GRASP, whether MONITOR was reached, the summed and the largest pad
force at `grasp_reached` and 1 s later, the servo position at contact and at grasp, and the
states the grasp ended in. Standard library only.
"""
import csv
import glob
import os
import sys


def load(path):
    meta, rows = {}, []
    with open(path, newline="") as f:
        lines = f.read().splitlines()
    data = [l for l in lines if l and not l.startswith("#")]
    for l in lines:
        if l.startswith("# started:"):
            for part in l[2:].split("|"):
                if ":" in part:
                    k, v = part.split(":", 1)
                    meta[k.strip()] = v.strip()
    if not data:
        return meta, rows
    for r in csv.DictReader(data):
        rows.append(r)
    return meta, rows


def forces(r):
    v = [float(r[f"f{i}"]) for i in range(1, 7) if r.get(f"f{i}")]
    return (sum(v), max(v)) if v else (0.0, 0.0)


def summarise(path, objects):
    name = os.path.basename(path)
    meta, rows = load(path)
    if not rows:
        return None
    t = lambda r: int(r["t_ms"])
    dur = {}
    for a, b in zip(rows, rows[1:]):
        key = a["state"] + ("/" + a["step"] if a["state"] == "PRE_TOUCH" else "")
        dur[key] = dur.get(key, 0) + (t(b) - t(a))
    reached = next((r for r in rows if r["event"] == "grasp_reached"), None)
    contact = next((r for r in rows if r["event"].startswith("contact:")), None)
    later = None
    if reached:
        later = next((r for r in rows if t(r) >= t(reached) + 1000), None)
    cls = next((r["cls"] for r in rows if r["state"] in ("PRE_TOUCH", "GRASP", "MONITOR") and r["cls"]), "")
    states = []
    for r in rows:
        if r["event"].startswith("state:"):
            states.append(r["event"][6:])
    fmt = lambda s: f"{s / 1000:.1f}"
    fr = lambda r: "%.2f / %.2f" % forces(r) if r else "-"
    return {
        "file": name, "object": objects.get(name, name.split("_")[2] if name.count("_") >= 3 else "?"),
        "result": meta.get("result", "?"), "class": cls or "-",
        "START s": fmt(dur.get("START", 0)), "approach s": fmt(dur.get("PRE_TOUCH/approach", 0)),
        "slow s": fmt(dur.get("PRE_TOUCH/slow", 0)), "GRASP s": fmt(dur.get("GRASP", 0)),
        "MONITOR": "yes" if any(r["state"] == "MONITOR" for r in rows) else "NO",
        "sum/max N at grasp": fr(reached), "sum/max N +1 s": fr(later),
        "pos contact": contact["pos"] if contact else "-", "pos grasp": reached["pos"] if reached else "-",
        "states": " > ".join(states),
    }


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    objects = {}
    if len(sys.argv) > 2 and os.path.exists(sys.argv[2]):
        with open(sys.argv[2], newline="") as f:
            for r in csv.DictReader(f):
                if r.get("file") and r.get("object"):
                    objects[r["file"].strip()] = r["object"].strip()
    out = [s for p in sorted(glob.glob(os.path.join(sys.argv[1], "grasp_*.csv"))) if not p.endswith("_tof.csv")
           for s in [summarise(p, objects)] if s]
    if not out:
        print("no grasp recordings found")
        return 1
    cols = list(out[0].keys())
    print("| " + " | ".join(cols) + " |")
    print("|" + "---|" * len(cols))
    for s in out:
        print("| " + " | ".join(str(s[c]) for c in cols) + " |")
    n = len(out); ok = sum(1 for s in out if s["MONITOR"] == "yes")
    print(f"\n{ok}/{n} grasps reached MONITOR.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
