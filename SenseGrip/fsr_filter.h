// Per-channel FSR force filters (none, moving average, median, EMA, 1-D Kalman). Pure C++.
#pragma once
#include <stdint.h>

enum class FilterKind : uint8_t { NONE = 0, MOVING_AVG = 1, MEDIAN = 2, EMA = 3, KALMAN = 4 };
struct FilterParams { FilterKind kind; uint8_t window; float alpha; float q; float r; };
constexpr uint8_t FILTER_MAX_WINDOW = 20;
struct ChannelFilter { float buf[FILTER_MAX_WINDOW]; uint8_t count, head; float y, p; bool init; };

void  filterReset(ChannelFilter& f);
// One sample in, one filtered sample out. NaN in -> NaN out, state untouched.
float filterStep(ChannelFilter& f, const FilterParams& p, float x);
// Approximate group delay: MA/median (N-1)/2*T, EMA (1-a)/a*T, Kalman via steady-state gain.
float filterGroupDelayMs(const FilterParams& p, float samplePeriodMs);
// Window actually used: moving average 2..20; median odd, 3..9; others 1.
uint8_t filterEffectiveWindow(const FilterParams& p);
