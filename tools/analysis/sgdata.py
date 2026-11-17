"""Load SenseGrip dashboard recordings and build labelled episodes from the operator marks.

A recording is the pair the dashboard downloads (docs/RECORDING_PROTOCOL.md §5):
  <name>.csv      `#` comment lines, the LOG_CSV_HEADER line, 100 Hz rows (SenseGrip/log_core.cpp)
  <name>_tof.csv  one `#` line, TOF_CSV_HEADER, raw SEN0628 frame sets (optional)
plus an optional operator notes file `file,object,trial,notes` anywhere in the folder.

Episodes (spec 2026-11-17-phase4b §7):
  slip         mark:pull|twist|slide  -> next mark:end, else +3 s (cut at the next start mark)
  disturbance  mark:other             -> next mark:end, else +2 s            (negative)
  quiet        detector-active MONITOR time outside every episode window   (negative)
Scoring window of an episode = [mark - tol, end + tol], tol = 0.5 s.
"""
from __future__ import annotations

import csv
import io
import os
import re
from dataclasses import dataclass, field

import numpy as np
import pandas as pd

MAIN_HEADER = ("seq,t_ms,state,step,armed,pos,speed,current_mA,load_pct,f1,f2,f3,f4,f5,f6,r1,r2,r3,r4,r5,r6,"
               "c1,c2,c3,tof_seq,obj_valid,width_mm,range_mm,height_mm,rad1,rad2,rad3,cls,shape,event").split(",")
TOF_HEADER = ["seq", "t_ms", "row_seq", "pos", "valid"] + [f"s{s}_z{z:02d}" for s in (1, 2, 3) for z in range(64)]
STATES = ["IDLE", "START", "PRE_TOUCH", "GRASP", "MONITOR", "OPEN", "FAULT"]
TEXT_COLS = ("state", "step", "cls", "shape", "event")
SLIP_TYPES = ("pull", "twist", "slide")
NAME_RE = re.compile(r"^(grasp|manual)_(\d+)(?:_([A-Za-z]+))?_(\d{8}-\d{6})\.csv$")

SLIP_DEFAULT_S = 3.0      # slip episode length when no end mark follows
DIST_DEFAULT_S = 2.0      # disturbance episode length when no end mark follows
TOL_S = 0.5               # scoring tolerance around an episode


@dataclass
class Episode:
    rec: str                # recording name
    kind: str               # "slip" | "disturbance"
    type: str               # pull | twist | slide | other
    t_mark: int             # ms (t_ms of the mark row)
    t_end: int              # ms
    end_by: str             # "end" | "timeout" | "next_mark" | "recording_end"
    scorable: bool          # mark fell inside MONITOR (the detector can only fire there)
    why_not: str = ""

    def window(self, tol_s: float = TOL_S) -> tuple[int, int]:
        return self.t_mark - int(tol_s * 1000), self.t_end + int(tol_s * 1000)


@dataclass
class Recording:
    name: str
    path: str
    kind: str                       # grasp | manual
    number: int
    cls: str                        # class from the file name ('' when absent)
    comments: list[str]
    rows: pd.DataFrame
    tof: pd.DataFrame | None
    tof_path: str | None
    object: str = ""
    trial: str = ""
    notes: str = ""
    gap_rows: int = 0               # missing seq numbers inside the file
    gap_flag: bool = False          # dashboard comment says GAP
    synthetic: bool = False
    slip_settings: dict = field(default_factory=dict)   # `# slip: slipXxx=v,...` comment -> {param: value}
    bad_lines: int = 0
    episodes: list[Episode] = field(default_factory=list)
    _arrays: dict | None = None

    # --- derived -------------------------------------------------------------
    @property
    def duration_s(self) -> float:
        t = self.rows["t_ms"].to_numpy()
        return float(t[-1] - t[0]) / 1000.0 if len(t) else 0.0

    def marks(self) -> list[tuple[int, int, str]]:
        """(seq, t_ms, type) for every mark:<type> event, in file order."""
        ev = self.rows["event"]
        m = ev.str.startswith("mark:")
        r = self.rows[m]
        return [(int(s), int(t), e[5:]) for s, t, e in zip(r["seq"], r["t_ms"], r["event"])]

    def events(self, prefix: str) -> pd.DataFrame:
        return self.rows[self.rows["event"].str.startswith(prefix)]

    def monitor_s(self) -> float:
        return float((self.rows["state"] == "MONITOR").sum()) * 0.01

    def has_monitor(self) -> bool:
        return bool((self.rows["state"] == "MONITOR").any())

    def data_warnings(self) -> list[str]:
        """What the replay tool also warns about: seq jumps, MONITOR rows without any frame."""
        w = []
        if self.bad_lines:
            w.append(f"{self.bad_lines} malformed lines dropped")
        if self.gap_rows:
            seq = self.rows["seq"].to_numpy()
            w.append(f"seq jumps: {int((np.diff(seq) > 1).sum())} ({self.gap_rows} rows missing)")
        elif self.gap_flag:
            w.append("dashboard marked GAP (rows missing before/after the file)")
        mon = self.rows["state"].to_numpy() == "MONITOR"
        if mon.any():
            nofr = mon & (self.arrays()["last_frame"] < 0)
            if self.tof is None:
                w.append("no _tof file: ToF feature cannot fire")
            elif nofr.any():
                w.append(f"{int(nofr.sum())} MONITOR rows without any frame")
        elif self.kind == "manual" or len(self.marks()):
            w.append("not replayable (no MONITOR)")
        return w

    def arrays(self) -> dict:
        """Numpy views used by the detector (cached)."""
        if self._arrays is None:
            self._arrays = detector_arrays(self)
        return self._arrays


# ---------------------------------------------------------------------------------------------------------------
def _read_csv_skip_comments(path: str, expected: list[str] | None) -> tuple[list[str], pd.DataFrame]:
    comments, body = [], []
    with open(path, "r", encoding="utf-8", errors="replace", newline="") as fh:
        for line in fh:
            s = line.rstrip("\r\n")
            if not s.strip():
                continue
            if s.lstrip().startswith("#"):
                comments.append(s.lstrip()[1:].strip())
            else:
                body.append(s)
    if not body:
        raise ValueError(f"{path}: no header line")
    header = body[0].split(",")
    nf = len(header)
    good = [body[0]] + [b for b in body[1:] if b.count(",") == nf - 1]   # drop truncated / garbled lines
    if expected is not None and header != expected:
        missing = [c for c in expected if c not in header]
        if missing:
            raise ValueError(f"{path}: header lacks columns {missing[:5]}")
    text_cols = {c: str for c in TEXT_COLS if c in header}
    df = pd.read_csv(io.StringIO("\n".join(good) + "\n"), dtype=text_cols, keep_default_na=False,
                     na_values={c: [""] for c in header if c not in text_cols}, on_bad_lines="skip",
                     engine="c", low_memory=False)
    for c in text_cols:
        df[c] = df[c].fillna("").astype(str)
    df.attrs["bad_lines"] = len(body) - len(good)
    return comments, df


def load_recording(path: str) -> Recording:
    base = os.path.basename(path)
    m = NAME_RE.match(base)
    kind, number, cls = ("manual" if base.startswith("manual") else "grasp"), 0, ""
    if m:
        kind, number, cls = m.group(1), int(m.group(2)), (m.group(3) or "")
    comments, rows = _read_csv_skip_comments(path, MAIN_HEADER)
    # rows the parser could not use (truncated last line, garbage) -> drop; keep file order
    rows = rows[pd.to_numeric(rows["seq"], errors="coerce").notna() & pd.to_numeric(rows["t_ms"], errors="coerce").notna()]
    rows = rows.astype({"seq": "int64", "t_ms": "int64"}).reset_index(drop=True)
    seq = rows["seq"].to_numpy()
    gap_rows = int(np.clip(np.diff(seq) - 1, 0, None).sum()) if len(seq) > 1 else 0
    gap_flag = any("GAP" in c for c in comments)
    slip_settings = parse_slip_comment(comments)
    synthetic = any("synthetic" in c.lower() for c in comments if c.lower().startswith("firmware"))
    tof_path = path[:-4] + "_tof.csv"
    tof = None
    if os.path.exists(tof_path):
        _, tof = _read_csv_skip_comments(tof_path, None)
        tof = tof.dropna(subset=["row_seq"]).reset_index(drop=True)
        zc = [c for c in TOF_HEADER[5:] if c in tof.columns]
        if len(zc) != 192:
            raise ValueError(f"{tof_path}: expected 192 zone columns, got {len(zc)}")
        tof[zc] = tof[zc].fillna(0)
    else:
        tof_path = None
    return Recording(name=base, path=path, kind=kind, number=number, cls=cls, comments=comments, rows=rows,
                     tof=tof, tof_path=tof_path, gap_rows=gap_rows, gap_flag=gap_flag, synthetic=synthetic,
                     slip_settings=slip_settings, bad_lines=int(rows.attrs.get("bad_lines", 0)))


def parse_slip_comment(comments: list[str]) -> dict:
    """`# slip: slipFsrDropFrac=0.3,slipMode=2,...` (detector settings at recording time) -> {fsrDropFrac: 0.3, ...}."""
    out = {}
    for c in comments:
        if not c.lower().startswith("slip:"):
            continue
        for kv in c[5:].replace(";", ",").split(","):
            if "=" not in kv:
                continue
            k, v = (x.strip() for x in kv.split("=", 1))
            if k.startswith("slip") and len(k) > 4:
                k = k[4].lower() + k[5:]
            try:
                out[k] = float(v)
            except ValueError:
                pass
    return out


def find_notes(folder: str) -> str | None:
    """A notes file: any .csv/.txt in the folder whose first line starts with `file,object`."""
    for fn in sorted(os.listdir(folder)):
        if not fn.lower().endswith((".csv", ".txt")) or NAME_RE.match(fn) or fn.endswith("_tof.csv"):
            continue
        p = os.path.join(folder, fn)
        try:
            with open(p, encoding="utf-8", errors="replace") as fh:
                first = fh.readline().strip().lower().replace(" ", "")
        except OSError:
            continue
        if first.startswith("file,object"):
            return p
    return None


def load_notes(path: str | None) -> dict[str, dict]:
    if not path:
        return {}
    out = {}
    with open(path, encoding="utf-8", errors="replace", newline="") as fh:
        for r in csv.DictReader(fh, restkey="_extra"):
            extra = r.pop("_extra", None) or []              # unquoted commas in the notes column
            r = {(k or "").strip().lower(): (v or "").strip() for k, v in r.items()}
            if extra:
                r["notes"] = ",".join([r.get("notes", "")] + [e.strip() for e in extra])
            f = os.path.basename(r.get("file", ""))
            if f:
                out[f] = r
    return out


def infer_trial(rec: Recording) -> str:
    types = {t for _, _, t in rec.marks() if t != "end"}
    if not types:
        return "A quiet"
    if types == {"other"}:
        return "B disturbances"
    letters = {"pull": "C pull", "twist": "D twist", "slide": "E slide"}
    slips = sorted(types & set(SLIP_TYPES))
    if len(slips) == 1:
        return letters[slips[0]]
    return "mixed " + "+".join(slips)


def label_episodes(rec: Recording) -> list[Episode]:
    """Slip / disturbance episodes from the marks (see module doc)."""
    marks = rec.marks()
    t = rec.rows["t_ms"].to_numpy()
    state = rec.rows["state"].to_numpy()
    t_last = int(t[-1]) if len(t) else 0
    eps = []
    for i, (seq, tm, typ) in enumerate(marks):
        if typ == "end":
            continue
        kind = "slip" if typ in SLIP_TYPES else "disturbance"
        dflt = SLIP_DEFAULT_S if kind == "slip" else DIST_DEFAULT_S
        t_end, end_by = tm + int(dflt * 1000), "timeout"
        for _, tm2, typ2 in marks[i + 1:]:
            if typ2 == "end":
                t_end, end_by = tm2, "end"
                break
            if tm2 < t_end:
                t_end, end_by = tm2, "next_mark"
            break
        if end_by == "timeout" and t_end > t_last:
            t_end, end_by = t_last, "recording_end"
        k = int(np.searchsorted(t, tm, side="right") - 1)
        st = state[max(k, 0)] if len(state) else ""
        scorable = st == "MONITOR"
        if scorable:
            why = ""
        elif not (state == "MONITOR").any():
            why = "not replayable (no MONITOR)" + (" — manual recording" if rec.kind == "manual" else "")
        else:
            why = f"mark in state {st or '?'}"
        eps.append(Episode(rec.name, kind, typ if typ in SLIP_TYPES or typ == "other" else "other", int(tm), int(t_end),
                           end_by, scorable, why))
    return eps


SKIPPED: list[tuple[str, str]] = []      # (file, why) of the last load_dataset call


def load_dataset(folder: str, notes_path: str | None = None) -> list[Recording]:
    """All main recordings in `folder` (sorted by name), with notes, object and episodes. Files that look like
    recordings but cannot be used (browser duplicates `x (1).csv`, unreadable files) are skipped and listed in
    `SKIPPED`."""
    SKIPPED.clear()
    cand = sorted(f for f in os.listdir(folder) if f.lower().endswith(".csv") and f.startswith(("grasp_", "manual_")))
    notes = load_notes(notes_path or find_notes(folder))
    recs = []
    for f in cand:
        if "_tof" in f:
            if not f.endswith("_tof.csv"):
                SKIPPED.append((f, "frames file with an unexpected name (browser duplicate?)"))
            continue
        if not NAME_RE.match(f):
            SKIPPED.append((f, "name does not match grasp_<n>_<class>_<stamp>.csv (browser duplicate?)"))
            continue
        try:
            r = load_recording(os.path.join(folder, f))
            if not len(r.rows):
                raise ValueError("no data rows")
        except Exception as e:                       # noqa: BLE001 - one bad file must not stop the analysis
            SKIPPED.append((f, f"unreadable: {e}"))
            continue
        n = notes.get(f, {})
        r.object = (n.get("object") or r.cls or "unknown").strip().lower()
        r.trial = n.get("trial") or infer_trial(r)
        r.notes = n.get("notes", "")
        if "manual" in r.notes.lower() and r.kind != "manual":
            r.notes += " [notes say manual]"
        r.episodes = label_episodes(r)
        recs.append(r)
    return recs


# ---------------------------------------------------------------------------------------------------------------
def detector_arrays(rec: Recording) -> dict:
    """Per-row inputs of the slip detector, as the replay tool feeds them (frames at row seq S = every frame with
    row_seq <= S not yet fed)."""
    r = rec.rows
    n = len(r)

    def col(c, dt=np.float32):
        return pd.to_numeric(r[c], errors="coerce").to_numpy(dtype=np.float64).astype(dt)

    def fixed(c, scale):
        """As the firmware feeds the detector (slipInputFromRow): logged int (mN, 0.1 mm) / scale in float32."""
        v = pd.to_numeric(r[c], errors="coerce").to_numpy(dtype=np.float64)
        ok = ~np.isnan(v)
        out = np.full(len(v), np.nan, np.float32)
        out[ok] = np.rint(v[ok] * scale).astype(np.float32) / np.float32(scale)
        return out

    a = {
        "seq": r["seq"].to_numpy(np.int64),
        "t": r["t_ms"].to_numpy(np.int64),
        "state": r["state"].to_numpy(),
        "monitor": (r["state"] == "MONITOR").to_numpy(),
        "force": np.stack([fixed(f"f{i}", 1000) for i in range(1, 7)], axis=1),
        "obj": (col("obj_valid", np.float64) != 0) & ~np.isnan(col("obj_valid", np.float64)),
        "range": fixed("range_mm", 10), "height": fixed("height_mm", 10),
        "rad": np.stack([fixed(f"rad{i}", 10) for i in range(1, 4)], axis=1),
    }
    last = np.full(n, -1, np.int64)
    new = np.zeros(n, bool)
    if rec.tof is not None and len(rec.tof):
        rs = rec.tof["row_seq"].to_numpy(np.int64)
        # frames in file order; frame j is fed at the first row with seq >= row_seq[j] (and after frame j-1)
        fed_at = np.searchsorted(a["seq"], rs, side="left")
        fed_at = np.maximum.accumulate(fed_at)
        ok = fed_at < n
        if ok.any():
            idx = np.nonzero(ok)[0]
            new[fed_at[idx]] = True
            last_at = np.full(n, -1, np.int64)
            # the latest frame fed at a row is the highest frame index with that fed_at
            np.maximum.at(last_at, fed_at[idx], idx)    # latest frame fed at each row
            last = np.maximum.accumulate(last_at)
        zc = TOF_HEADER[5:]
        a["frames"] = rec.tof[zc].to_numpy(dtype=np.float64).clip(0, 65535).astype(np.uint16).reshape(-1, 3, 64)
        a["fvalid"] = rec.tof["valid"].fillna(0).to_numpy(np.int64)
        a["ftime"] = rec.tof["t_ms"].to_numpy(np.int64)
    else:
        a["frames"] = np.zeros((0, 3, 64), np.uint16)
        a["fvalid"] = np.zeros(0, np.int64)
        a["ftime"] = np.zeros(0, np.int64)
    a["last_frame"] = last
    a["frame_new"] = new
    return a
