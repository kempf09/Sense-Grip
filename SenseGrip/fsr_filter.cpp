#include "fsr_filter.h"
#include <cmath>

void filterReset(ChannelFilter& f) {
  for (auto& v : f.buf) v = 0;
  f.count = 0; f.head = 0; f.y = 0; f.p = 0; f.init = false;
}

uint8_t filterEffectiveWindow(const FilterParams& p) {
  uint8_t w = p.window;
  switch (p.kind) {
    case FilterKind::MOVING_AVG: return w < 2 ? 2 : w > FILTER_MAX_WINDOW ? FILTER_MAX_WINDOW : w;
    case FilterKind::MEDIAN:
      if (w < 3) w = 3;
      if (!(w & 1)) w++;
      return w > 9 ? 9 : w;
    default: return 1;
  }
}

namespace {
float kalmanGain(float q, float r) {
  const float pm = (q + std::sqrt(q * q + 4 * q * r)) / 2;
  return pm / (pm + r);
}
}  // namespace

float filterStep(ChannelFilter& f, const FilterParams& p, float x) {
  if (std::isnan(x)) return x;
  switch (p.kind) {
    case FilterKind::MOVING_AVG:
    case FilterKind::MEDIAN: {
      f.buf[f.head] = x;
      f.head = (uint8_t)((f.head + 1) % FILTER_MAX_WINDOW);
      if (f.count < FILTER_MAX_WINDOW) f.count++;
      const uint8_t n = f.count < filterEffectiveWindow(p) ? f.count : filterEffectiveWindow(p);
      float last[FILTER_MAX_WINDOW];
      for (uint8_t i = 0; i < n; i++)
        last[i] = f.buf[(f.head + FILTER_MAX_WINDOW - 1 - i) % FILTER_MAX_WINDOW];
      if (p.kind == FilterKind::MOVING_AVG) {
        float s = 0;
        for (uint8_t i = 0; i < n; i++) s += last[i];
        return s / n;
      }
      for (uint8_t i = 1; i < n; i++) {           // insertion sort, n <= 9
        float v = last[i]; int j = i - 1;
        while (j >= 0 && last[j] > v) { last[j + 1] = last[j]; j--; }
        last[j + 1] = v;
      }
      return (n & 1) ? last[n / 2] : 0.5f * (last[n / 2 - 1] + last[n / 2]);
    }
    case FilterKind::EMA:
      if (!f.init) { f.y = x; f.init = true; return x; }
      f.y += p.alpha * (x - f.y);
      return f.y;
    case FilterKind::KALMAN: {
      if (!f.init) { f.y = x; f.p = p.r; f.init = true; return x; }
      f.p += p.q;
      const float k = f.p / (f.p + p.r);
      f.y += k * (x - f.y);
      f.p *= (1 - k);
      return f.y;
    }
    default: return x;
  }
}

float filterGroupDelayMs(const FilterParams& p, float t) {
  switch (p.kind) {
    case FilterKind::MOVING_AVG:
    case FilterKind::MEDIAN: return (filterEffectiveWindow(p) - 1) / 2.0f * t;
    case FilterKind::EMA: return p.alpha > 0 ? (1 - p.alpha) / p.alpha * t : 0;
    case FilterKind::KALMAN: { const float k = kalmanGain(p.q, p.r); return k > 0 ? (1 - k) / k * t : 0; }
    default: return 0;
  }
}

bool filterParamsEqual(const FilterParams& a, const FilterParams& b) {
  return a.kind == b.kind && a.window == b.window && a.alpha == b.alpha && a.q == b.q && a.r == b.r;
}
