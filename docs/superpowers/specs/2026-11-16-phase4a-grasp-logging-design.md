# Phase 4a — Grasp data logging (CSV) — design

Date: 2026-11-16 · Branch: `feature/continuous-approach-touch-window` · Builds on: phase 1–3 specs
Status: draft for approval

## 1. Why
The thesis needs data, not only video: force curves per pad, raw vs filtered FSR, motion and
measured object size per grasp, and (phase 4b) recordings of deliberate slips to set the slip
thresholds from measurements instead of guesses. The operator asked Claude to decide what is
collected; storage in the browser was the operator's wish.

## 2. Goals / non-goals / anti-goals
- **Goals:** every control step (100 Hz) recorded with no gaps while the dashboard is open; one
  recording per grasp, kept in the browser across page reloads, downloadable as CSV; manual
  recording for non-grasp experiments (e.g. filter comparison, pushing pads by hand).
- **Non-goals:** writing logs to the gripper's flash; analysis or plotting inside the dashboard
  (beyond a list of recordings); slip detection itself (phase 4b).
- **Anti-goals:** logging must never slow the control loop (no flash writes, no blocking); a
  recording must never silently miss rows (gaps are marked).

## 3. Approaches (operator picked B)
| | How | Trade-off |
|---|---|---|
| A | Browser records what it already polls (`/api/status`, ~5 Hz) | No firmware change; too coarse for force curves and slips |
| **B** | **Firmware ring buffer in PSRAM at 100 Hz; the browser fetches new rows a few times a second into IndexedDB; per-grasp recordings; CSV download** | Needs the page open during the experiment (the ring covers ~5 min of disconnection) |
| C | CSV files per grasp in LittleFS | ~800 KB free = few grasps; flash writes stall the loop |

## 4. Design
### 4.1 Row (one per 10 ms control step, written by the control task after the FSR sample)
| Column | Unit / values |
|---|---|
| `seq` | row counter since boot (gap detection) |
| `t_ms` | `millis()` |
| `state`, `step`, `armed` | FSM state name; PRE_TOUCH step (`approach`/`slow`/`-`); 0/1 |
| `pos`, `speed`, `current_mA`, `load_pct` | servo logical position (counts), speed, current, load |
| `f1..f6` | filtered force (N), printed channel order FSR1..FSR6 |
| `r1..r6` | raw force (N), same order |
| `c1..c3` | debounced contact per finger (0/1) |
| `tof_seq` | SEN0628 frame-set counter (changes when a new frame set was measured) |
| `obj_valid`, `width_mm`, `range_mm`, `height_mm` | smoothed object measurement |
| `rad1..rad3` | per-sensor object radius (mm, empty when not seen) |
| `cls`, `shape` | effective class (`small`…`delicate`, empty when no plan), shape name |
| `event` | text, empty on most rows: `state:<NEW>`, `contact:F<n>:<proximal|distal|both>`, `grasp_reached`, later `slip:...` |

Binary in the ring (≈64 B: forces as int16 mN, mm as int16), CSV only when served.

### 4.2 Firmware
- `log_core` (pure, host-tested): fixed-size ring of rows (`LOG_CAP` = 30 000 ≈ 5 min, in PSRAM),
  single writer (control task) / single reader (web task): `logPush(row)`, `logRead(fromSeq, max,
  out)` returning rows still in the ring and the oldest available seq (reader detects overwrite
  → gap). Lock-free: each row carries its own `seq`; the reader copies without holding the
  spinlock and drops any copied row whose `seq` is not the one it expected (overwritten while
  copying). Events are stored as a small code + argument, turned into text when served. Row → CSV line formatter (fixed column order, header line constant).
- `control.cpp`: builds and pushes one row per FSR sample (100 Hz); events from FSM state change,
  contact pattern changes and grasp reached.
- `GET /api/log?from=<seq>&max=<n≤600>` → `text/csv`: header line (only when `from=0` or
  `header=1`), then rows; response headers `X-Log-Next` (next seq to ask for) and `X-Log-Oldest`.
  If `from` < oldest, the reply starts at oldest and the browser marks a gap.

### 4.3 Browser (dashboard)
- Collector: while the page is open, fetch `/api/log?from=<next>` every 250 ms; keep `next` in
  IndexedDB so a reload continues where it stopped (rows still in the ring are recovered).
- **Recordings** (new panel on the Combined tab, also reachable from the SEN0628/FSR tabs as a
  link): automatic recording from the row whose state becomes START to 1 s after OPEN → IDLE
  (or FAULT + 1 s). Each recording is stored in IndexedDB with: start time, duration, class,
  width, final state, number of rows, gap flag. List with **Download CSV**, **Delete**,
  **Download all** (one CSV file per recording).
- **Manual recording:** Start / Stop button records every row in between (any state).
- Storage budget: ~150 B/row text → a 20 s grasp ≈ 300 KB; IndexedDB comfortably holds hundreds.
  A "Clear all" button. Recordings are named `grasp_<n>_<class>_<YYYYMMDD-HHMMSS>.csv` /
  `manual_<n>_...csv`.
- CSV: comma separated, `.` decimal point, one header line, UTF-8; the first lines are `#`
  comments with firmware build, config summary (class profile used, filter, contact force),
  so a file is self-describing (the page reads `/api/config` and the header build stamp for these).

### 4.4 Builder's calls
Ring size, fetch interval, exact rounding (forces 0.001 N, mm 0.1), panel placement, the
self-describing comment lines.

## 5. Verification
Host tests: ring wrap/overwrite/gap reporting, CSV formatting (columns, empty fields, events).
Hardware: record a grasp, check row count ≈ 100 × duration with no gap; reload the page
mid-grasp and confirm the recording continues; open the CSV in Excel/Python.
