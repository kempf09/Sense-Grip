# Phase 1 — FSR Filtering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Selectable per-channel FSR filters (none, moving average, median, EMA, 1-D Kalman) at 100 Hz, used by the FSM, with raw and filtered force shown side by side on a live chart.

**Architecture:** A pure, host-tested `fsr_filter` module holds the filter state machines and the group-delay formula. `fsr.cpp` keeps one filter state per channel, filters calibrated force each sample, and publishes both raw and filtered force. Filter settings are new `GripConfig` fields (config version 5, size-based migration); the dashboard FSR tab gets a selector and a canvas chart.

**Tech Stack:** Arduino-ESP32 3.x, existing SenseGrip modules, host tests via `make -C tests` (g++ C++17, `tests/check.h`), vanilla JS in `SenseGrip/web_pages.h`.

**Spec:** `docs/superpowers/specs/2026-11-15-sensing-homing-perception-design.md` §5 (Phase 1). Phases 2 and 3 get their own plans after phase 1 is tested on hardware.

## Global Constraints

- FSR sampling period **10 ms (100 Hz)**; ADC oversampling stays 16 (drop to 8 only if one sample pass measures > 4 ms on hardware).
- Filters: 0 none · 1 moving average N 2–20 (default 5) · 2 median N ∈ {3,5,7,9} (default 5) · 3 EMA α 0.05–1.0 (default 0.3) · 4 Kalman Q, R (defaults 0.01, 0.05).
- **Default filter: median, window 5.**
- One filter kind/parameter set for all six channels.
- FSM, contact debounce and aggregates use **filtered** force; **the force-limit input is max(raw, filtered)** per channel.
- `/api/status` adds `fsr.forceRaw[6]`; `fsr.force[6]` becomes the filtered value.
- New config fields are appended after `fsrEnabled`; config version 5; v3/v4 blobs load (filter fields default), v1–v3 still invalidate limits, v4 keeps limits.
- Pure files (`fsr_filter.*`) must not include Arduino headers.
- Commit trailer lines as in previous tasks (`Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` and the `Claude-Session:` line); push to `feature/continuous-approach-touch-window`.

## Review Focus

1. **Filter changed while running** → all channel states reset; the first output after a change equals the first input (no stale history). Test `filter_reset_on_param_change` (Task 1).
2. **Uncalibrated channel (NaN force)** → output NaN, history not poisoned: after NaN then 1.0, output is 1.0-based, never NaN. Test `filter_nan_passthrough` (Task 1).
3. **Spike above the force limit removed by the median** → safety still sees it via max(raw, filtered). Test `fsr_limit_uses_raw_when_filter_hides_spike` (Task 3).
4. **Even median window from config (e.g. 4)** → treated as next odd (5), capped at 9. Test `filter_median_even_window_rounds_up` (Task 1).
5. **Window larger than the buffer (e.g. 50)** → clamped by `clampConfig` to 20 (MA) / 9 (median) without overrun. Test `config_filter_fields_clamped` (Task 2) + `filter_window_capped` (Task 1).

---

### Task 1: Pure filter module

**Files:**
- Create: `SenseGrip/fsr_filter.h`, `SenseGrip/fsr_filter.cpp`
- Test: `tests/test_fsr_filter.cpp`

**Interfaces:**
- Produces:
```cpp
enum class FilterKind : uint8_t { NONE = 0, MOVING_AVG = 1, MEDIAN = 2, EMA = 3, KALMAN = 4 };
struct FilterParams { FilterKind kind; uint8_t window; float alpha; float q; float r; };
constexpr uint8_t FILTER_MAX_WINDOW = 20;
struct ChannelFilter { float buf[FILTER_MAX_WINDOW]; uint8_t count, head; float y, p; bool init; };
void  filterReset(ChannelFilter& f);
float filterStep(ChannelFilter& f, const FilterParams& p, float x);   // NaN in -> NaN out, state untouched
float filterGroupDelayMs(const FilterParams& p, float samplePeriodMs);
uint8_t filterEffectiveWindow(const FilterParams& p);                  // MA: clamp 2..20; median: odd, 3..9
```
Group delay: MA and median `(N−1)/2 · T`; EMA `(1−α)/α · T`; Kalman: steady-state `P⁻ = (Q + √(Q² + 4QR))/2`, `K = P⁻/(P⁻+R)`, delay `(1−K)/K · T`; none 0. Kalman step: `p += q; k = p/(p+r); y += k(x−y); p *= (1−k)`, initialised `y = x, p = r` on the first sample. EMA: first sample `y = x`, then `y += α(x−y)`. Moving average and median work over the last `min(count, N)` samples (warm-up uses what is there).

- [ ] **Step 1: Write the failing tests** in `tests/test_fsr_filter.cpp`:
  - `filter_none_passthrough`: NONE returns the input exactly for 1.0, 2.5, 0.
  - `filter_moving_avg_step`: N=4, inputs 0,0,0,0,4,4,4,4 → outputs at steps 5..8 = 1,2,3,4.
  - `filter_median_rejects_spike`: N=5, inputs 1,1,1,10,1,1 → every output ≤ 1.0 + 1e-6.
  - `filter_median_even_window_rounds_up`: window 4 → `filterEffectiveWindow` = 5; window 12 → 9.
  - `filter_window_capped`: MA window 50 → effective 20; stepping 30 samples does not crash and averages the last 20.
  - `filter_ema_first_sample`: α=0.5, constant 2.0 → first output 2.0 (initialised to the first input), stays 2.0.
  - `filter_ema_step_half`: α=0.5, inputs 0 then 1 → second output 0.5.
  - `filter_kalman_converges`: Q=0.01, R=0.05, 200 samples of 1.0 after one 0 → final output within 0.01 of 1.0.
  - `filter_nan_passthrough`: median 5: inputs 1, NaN, 1 → middle output is NaN, third output is 1.0.
  - `filter_reset_on_param_change`: EMA α 0.1 fed 0×10, `filterReset`, input 5 → output 5.
  - `filter_group_delay`: T=10: MA5 → 20 ms; median5 → 20 ms; EMA α0.5 → 10 ms; none → 0; Kalman Q0.01 R0.05 → value within 0.5 ms of the formula above (compute expected in the test).
- [ ] **Step 2:** `make -C tests` — Expected: build fails (`fsr_filter.h` missing).
- [ ] **Step 3:** Implement `fsr_filter.cpp` (median: copy the last N into a small array and insertion-sort; no heap).
- [ ] **Step 4:** `make -C tests` — Expected: all pass, output pristine.
- [ ] **Step 5:** Commit `feat: FSR filter module (MA, median, EMA, Kalman) with group delay`.

### Task 2: Filter settings in the config

**Files:**
- Modify: `SenseGrip/config_core.h`, `SenseGrip/config_core.cpp`, `SenseGrip/config.cpp`, `SenseGrip/web.cpp` (FIELDS table)
- Test: `tests/test_config.cpp`

**Interfaces:**
- Consumes: `FilterKind` values (Task 1) — stored as `uint8_t`.
- Produces: `GripConfig` fields appended after `fsrEnabled`, in this order:
  `uint8_t fsrFilter; uint8_t fsrFilterWindow; float fsrFilterAlpha; float fsrKalmanQ; float fsrKalmanR;`
  and `constexpr unsigned GRIP_CONFIG_V4_SIZE = __builtin_offsetof(GripConfig, fsrFilter);`
  plus `FilterParams filterParamsOf(const GripConfig&)` declared in `config_core.h` (config_core.cpp includes `fsr_filter.h`).
- Defaults: `fsrFilter = 2` (median), `fsrFilterWindow = 5`, `fsrFilterAlpha = 0.3`, `fsrKalmanQ = 0.01`, `fsrKalmanR = 0.05`.
- Clamp: filter 0–4 (out of range → default 2); window 2–20; alpha 0.05–1.0; Q and R 1e-5–10 (non-finite → default).
- `config.cpp`: `CFG_VERSION = 5`; size map: v5 → `sizeof(GripConfig)`; v3, v4 → `GRIP_CONFIG_V4_SIZE`; v1, v2 → `GRIP_CONFIG_V2_SIZE`; limits invalidated for v1–v3 only (v4 limits kept).
- Web: add `F_(fsrFilter, T_U8), F_(fsrFilterWindow, T_U8), F_(fsrFilterAlpha, T_F), F_(fsrKalmanQ, T_F), F_(fsrKalmanR, T_F)`.

- [ ] **Step 1: Failing tests:** `config_filter_defaults` (values above; `!clampConfig(defaultConfig())`), `config_filter_fields_clamped` (filter 9 → 2, window 50 → 20, window 0 → 2, alpha 0 → 0.05, Q NaN → 0.01), `config_blob_sizes_ordered` (`GRIP_CONFIG_V2_SIZE == offsetof(GripConfig, fsrEnabled)`, `GRIP_CONFIG_V4_SIZE > GRIP_CONFIG_V2_SIZE`, `GRIP_CONFIG_V4_SIZE < sizeof(GripConfig)`), `config_filter_params_of` (`filterParamsOf(defaultConfig()).kind == FilterKind::MEDIAN`, window 5).
- [ ] **Step 2:** Run — FAIL. **Step 3:** Implement. **Step 4:** Run — PASS.
- [ ] **Step 5:** Commit `feat: FSR filter settings in config (v5)`.

### Task 3: Filtering in the sampling path at 100 Hz

**Files:**
- Modify: `SenseGrip/fsr.h`, `SenseGrip/fsr.cpp`, `SenseGrip/fsr_math.h`, `SenseGrip/fsr_math.cpp`, `SenseGrip/control.cpp`, `SenseGrip/web.cpp` (status JSON)
- Test: `tests/test_fsr.cpp`

**Interfaces:**
- Consumes: Task 1 (`ChannelFilter`, `filterStep`, `filterReset`), Task 2 (`filterParamsOf`).
- Produces:
  - `FsrSnapshot` gains `float forceRaw[6];` (`force[6]` = filtered) and `float limitForce;` (max over enabled channels of max(raw, filtered)).
  - `void fsrSample(float contactN, uint8_t samples, uint8_t enabledMask, const FilterParams& fp);` — resets all channel filters when `fp` differs from the previous call's (memcmp of the struct).
  - Pure helper in `fsr_math`: `float limitForceOf(const float raw[6], const float filtered[6], uint8_t enabledMask);` — max over enabled channels of max(raw, filtered), NaN ignored, 0 if none.
  - `control.cpp`: FSR sampled every **10 ms**; `in.maxForce = fs.limitForce`.
  - `/api/status` `fsr` object adds `"forceRaw":[…]` and `"filterDelayMs":<filterGroupDelayMs(fp, 10)>`.

- [ ] **Step 1: Failing tests** in `tests/test_fsr.cpp`: `fsr_limit_uses_raw_when_filter_hides_spike` (raw {0,0,5,0,0,0}, filtered {0,0,0.2,0,0,0}, mask 0x3F → 5.0), `fsr_limit_ignores_disabled_and_nan` (raw ch1 = 9 but mask without bit 1 → not 9; NaN entries ignored).
- [ ] **Step 2:** Run — FAIL. **Step 3:** Implement `limitForceOf`; wire `fsr.cpp` (filter per channel after `fsrForce`, keep `forceRaw`; masking/aggregates on filtered), `control.cpp` (10 ms, pass `filterParamsOf(c->cfg)`), `web.cpp`. **Step 4:** Run — PASS.
- [ ] **Step 5:** Push; CI `esp32` job compiles. Commit `feat: filtered FSR at 100 Hz, force limit on max(raw, filtered)`.

### Task 4: Dashboard — filter selector and live raw/filtered chart

**Files:**
- Modify: `SenseGrip/web_pages.h` (FSR tab)

**Interfaces:**
- Consumes: `/api/status` `fsr.force`, `fsr.forceRaw`, `fsr.filterDelayMs`; `/api/config` fields from Task 2.
- Produces: FSR tab panel "Filter":
  - selector Filter (None / Moving average / Median / EMA / Kalman) with only the relevant parameter inputs shown (window for MA/median, α for EMA, Q and R for Kalman), Save posts the changed fields to `/api/config`;
  - text "Delay ≈ X ms at 100 Hz" from `filterDelayMs`;
  - channel selector (printed channel numbers {2,1,3,4,5,6} by index) and a canvas chart of the last 10 s: raw (thin grey) and filtered (accent colour) force in N, auto-scaled y with 0 baseline, legend. Points appended on every status poll (200 ms); history in a ring buffer (no unbounded arrays).

- [ ] **Step 1:** Implement.
- [ ] **Step 2:** Extract the `<script>` and run `node --check` — no errors; raw string contains no `)HTML"`.
- [ ] **Step 3:** Smoke test with the scratchpad mock (mock `/api/status` with varying `force`/`forceRaw`): chart draws both series, changing the filter posts only `fsrFilter`/parameters, no console errors, phone width 390 px has no horizontal scroll.
- [ ] **Step 4:** Commit `feat: FSR filter selector and raw/filtered chart`.

### Task 5: Docs and hardware checklist

**Files:**
- Modify: `README.md`, `docs/HARDWARE_CHECKLIST.md`

- [ ] **Step 1:** README: FSR filtering section (filters, defaults, delay formula, force limit uses max(raw, filtered), 100 Hz), API field additions.
- [ ] **Step 2:** Checklist step: "FSR filters — press a pad by hand and tap it: compare raw vs each filter on the chart; note the delay readout; confirm the force limit still trips on a sharp press with the median filter on." Also: "check the control loop max (SEN0628 tab) stays below 10 ms with 100 Hz sampling."
- [ ] **Step 3:** `make -C tests` passes; commit `docs: FSR filtering`; push.
