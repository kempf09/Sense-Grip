# Phase 4a — Grasp Data Logging Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Record every 100 Hz control step into a PSRAM ring, serve it as CSV, and let the dashboard collect per-grasp and manual recordings in IndexedDB with CSV download.

**Architecture:** Pure `log_core` (row struct, ring with per-row seq, CSV formatting) host-tested; `control.cpp` pushes one row per FSR sample; `web.cpp` serves `GET /api/log`; the dashboard collects, segments and stores recordings.

**Tech Stack:** Arduino-ESP32 3.x C++17, FreeRTOS, PSRAM (`ps_malloc`); vanilla JS + IndexedDB in `web_pages.h`; host tests `make -C tests`.

**Spec:** `docs/superpowers/specs/2026-11-16-phase4a-grasp-logging-design.md`

## Global Constraints
- Logging never blocks the control loop: no flash writes, no spinlock held while copying rows.
- Columns exactly as spec §4.1, in that order; forces in printed channel order FSR1..FSR6 (FSR n = index `i` with `FSR_CHANNEL_NUMBER[i] == n`).
- `LOG_CAP` 30 000 rows; `/api/log` `max` ≤ 600; browser fetch every 250 ms.
- Rounding: forces 0.001 N, mm 0.1, current mA integer, load % 0.1.
- Commit trailers as in earlier plans; no PR.

## Review Focus
1. Reader asks for a seq already overwritten → reply starts at the oldest row and the browser marks the recording `gap` (test in Task 1: `log_read_after_overwrite_reports_oldest`).
2. Row overwritten while the reader copies → that row is dropped, never served with wrong data (Task 1: `log_read_drops_rows_overwritten_during_copy` via the seq check helper).
3. ESP32 reboot (seq restarts at 0) while the page is open → browser sees `X-Log-Next` < its `next`, resets `next` and closes any open recording with a gap (Task 3, manual hardware check; noted in the checklist).
4. NaN forces / object not measured → empty CSV fields, not "nan" (Task 1: `log_csv_empty_fields`).
5. Grasp that ends in FAULT → the recording closes 1 s after FAULT (Task 3 segmentation rule).

---

### Task 1: `log_core` — row, ring, CSV

**Files:** Create `SenseGrip/log_core.h`, `SenseGrip/log_core.cpp`; Test `tests/test_log.cpp`

**Interfaces (produces):**
```cpp
enum class LogEvent : uint8_t { NONE, STATE, CONTACT, GRASP_REACHED };
struct LogRow {
  uint32_t seq, tMs; uint8_t state, step, armed;          // State, PreTouchStep (+1; 0 = none)
  int32_t pos; int16_t speed, currentMa; int16_t loadPermille;
  int16_t fmN[6], rmN[6];                                 // printed order FSR1..6, mN; INT16_MIN = NaN
  uint8_t contacts;                                       // bit f = finger f+1
  uint16_t tofSeq; uint8_t objValid;
  int16_t width10, range10, height10, rad10[3];           // 0.1 mm; INT16_MIN = empty
  uint8_t cls, shape;                                     // cls 0..3, 255 = none; Shape
  LogEvent ev; uint8_t evArg, evArg2;                     // STATE: new State; CONTACT: finger 1..3, FirstContact
};
struct LogRing { LogRow* rows; uint32_t cap; volatile uint32_t next; };   // next = seq of the next push
void logInit(LogRing& r, LogRow* storage, uint32_t cap);
void logPush(LogRing& r, LogRow row);                     // sets row.seq = r.next, stores, then ++next
// Copies up to max rows starting at fromSeq (clamped to the oldest still held); returns count,
// sets oldest and nextOut (seq after the last copied). Rows whose seq differs from the expected
// one after copying (overwritten meanwhile) end the copy.
uint32_t logRead(const LogRing& r, uint32_t fromSeq, uint32_t max, LogRow* out, uint32_t& oldest, uint32_t& nextOut);
extern const char LOG_CSV_HEADER[];                        // spec §4.1 column names, comma separated, no newline
int logCsvLine(const LogRow& row, char* buf, int len);     // one line + '\n'; returns length
```
State/class/shape names come from `stateName`, `className`, `shapeName`; step text `approach`/`slow`/`-`; event text `state:<NAME>`, `contact:F<n>:<proximal|distal|both>`, `grasp_reached`.

- [ ] **Step 1: Failing tests** in `tests/test_log.cpp`: `log_push_read_basic` (push 5, read from 0 → 5 rows, seq 0..4, nextOut 5); `log_read_after_overwrite_reports_oldest` (cap 10, push 25, read from 3 → oldest 15, first row seq 15, count 10); `log_read_respects_max`; `log_csv_header_columns` (header has 35 comma-separated names starting `seq,t_ms,state,step,armed,pos` and ending `cls,shape,event`); `log_csv_line_values` (row with forces 1.234 N → `1.234`, width 62.3 → `62.3`, event STATE GRASP → `state:GRASP`, field count equals header); `log_csv_empty_fields` (INT16_MIN forces/radii → empty fields, cls 255 → empty).
- [ ] **Step 2:** `make -C tests` → FAIL (no header).
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** `make -C tests` → pass.
- [ ] **Step 5:** Commit `feat: log ring and CSV rows`.

### Task 2: Firmware — record rows, serve `/api/log`

**Files:** Modify `SenseGrip/control.cpp`, `SenseGrip/control.h`, `SenseGrip/web.cpp`

- `control.cpp`: allocate the ring in PSRAM at task start (`ps_malloc(LOG_CAP * sizeof(LogRow))`, malloc fallback with a smaller cap 3000); after each `fsrSample` build a row from servo/FSR/ToF/object/plan/FSM snapshots and `logPush`. Events: state change since the previous row (`STATE`), a finger's `first[]` changing from NONE (`CONTACT`), entering MONITOR from GRASP (`GRASP_REACHED`; one event per row, priority STATE > GRASP_REACHED > CONTACT, a pending lower-priority event goes in the next row).
- `control.h`: `const LogRing* controlLog();`.
- `web.cpp`: `GET /api/log?from=<seq>&max=<n>` (default max 300, cap 600, `header=1` or `from=0` adds the header line), `text/csv`, chunked, headers `X-Log-Next`, `X-Log-Oldest`, `X-Log-Boot` (a random 32-bit id chosen at boot, so the browser detects reboots), `Cache-Control: no-store`.
- [ ] **Step 1:** Implement; `make -C tests` still passes.
- [ ] **Step 2:** Push; CI esp32 compile green.
- [ ] **Step 3:** Commit `feat: record control steps and serve /api/log`.

### Task 3: Dashboard — collector, recordings, CSV download

**Files:** Modify `SenseGrip/web_pages.h`

- Collector: every 250 ms `GET /api/log?from=next&max=600`; parse lines; `X-Log-Boot` change or `X-Log-Next` < next → reset next to `X-Log-Oldest`, close any open recording with `gap`. `X-Log-Oldest` > requested from → mark open recording `gap`. Persist `{boot, next}` in IndexedDB (`sensegrip` DB, stores `meta`, `recs`).
- Segmentation: auto recording opens at the first row with `state=START`, closes 1 s (100 rows) after the state becomes IDLE following OPEN, or after FAULT; manual Start/Stop records all rows between. Recording object `{id, kind, start, rows:[lines], cls, width, final, gap}`; saved on close (and every 5 s while open).
- Panel "Recordings" on the Combined tab: Start/Stop manual, live row rate, list (name, duration, rows, class, width, final state, gap mark) with Download / Delete, Download all, Clear all. File names per spec. CSV = `#` comment lines (build from `S.diag.fw`, class profile, filter, contact force from `CFG`) + header + rows.
- [ ] **Step 1:** Implement; `node --check` on the extracted script; headless render with mocked `/api/log` (rows across START→OPEN→IDLE) shows one recording whose CSV has the header and rows.
- [ ] **Step 2:** Push; CI green. Commit `feat: recordings panel with CSV download`.

### Task 4: Docs
- README: Recordings section, `/api/log` row in the API table, `log_core` in the code map. Checklist steps 32–34: grasp recording row count ≈ 100 × duration without gap; reload mid-grasp keeps the recording; open a CSV in Excel/Python.
- [ ] Commit `docs: grasp logging`.
