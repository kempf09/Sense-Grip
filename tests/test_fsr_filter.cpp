#include "check.h"
#include "fsr_filter.h"
#include <cmath>

static FilterParams P(FilterKind k, uint8_t w = 5, float a = 0.3f, float q = 0.01f, float r = 0.05f) {
  return FilterParams{k, w, a, q, r};
}

TEST(filter_none_passthrough) {
  ChannelFilter f; filterReset(f);
  auto p = P(FilterKind::NONE);
  CHECK_NEAR(filterStep(f, p, 1.0f), 1.0f, 0);
  CHECK_NEAR(filterStep(f, p, 2.5f), 2.5f, 0);
  CHECK_NEAR(filterStep(f, p, 0.0f), 0.0f, 0);
}

TEST(filter_moving_avg_step) {
  ChannelFilter f; filterReset(f);
  auto p = P(FilterKind::MOVING_AVG, 4);
  for (int i = 0; i < 4; i++) filterStep(f, p, 0);
  CHECK_NEAR(filterStep(f, p, 4), 1, 1e-5);
  CHECK_NEAR(filterStep(f, p, 4), 2, 1e-5);
  CHECK_NEAR(filterStep(f, p, 4), 3, 1e-5);
  CHECK_NEAR(filterStep(f, p, 4), 4, 1e-5);
}

TEST(filter_median_rejects_spike) {
  ChannelFilter f; filterReset(f);
  auto p = P(FilterKind::MEDIAN, 5);
  const float in[] = {1, 1, 1, 10, 1, 1};
  for (float x : in) CHECK(filterStep(f, p, x) <= 1.0f + 1e-6f);
}

TEST(filter_median_even_window_rounds_up) {
  CHECK_EQ(filterEffectiveWindow(P(FilterKind::MEDIAN, 4)), (uint8_t)5);
  CHECK_EQ(filterEffectiveWindow(P(FilterKind::MEDIAN, 12)), (uint8_t)9);
  CHECK_EQ(filterEffectiveWindow(P(FilterKind::MEDIAN, 1)), (uint8_t)3);
}

TEST(filter_window_capped) {
  CHECK_EQ(filterEffectiveWindow(P(FilterKind::MOVING_AVG, 50)), (uint8_t)20);
  ChannelFilter f; filterReset(f);
  auto p = P(FilterKind::MOVING_AVG, 50);
  float y = 0;
  for (int i = 0; i < 30; i++) y = filterStep(f, p, i < 10 ? 100.0f : 1.0f);
  CHECK_NEAR(y, 1.0f, 1e-5);                    // last 20 samples are all 1
}

TEST(filter_ema_first_sample) {
  ChannelFilter f; filterReset(f);
  auto p = P(FilterKind::EMA, 5, 0.5f);
  CHECK_NEAR(filterStep(f, p, 2.0f), 2.0f, 1e-6);
  CHECK_NEAR(filterStep(f, p, 2.0f), 2.0f, 1e-6);
}

TEST(filter_ema_step_half) {
  ChannelFilter f; filterReset(f);
  auto p = P(FilterKind::EMA, 5, 0.5f);
  filterStep(f, p, 0);
  CHECK_NEAR(filterStep(f, p, 1), 0.5f, 1e-6);
}

TEST(filter_kalman_converges) {
  ChannelFilter f; filterReset(f);
  auto p = P(FilterKind::KALMAN, 5, 0.3f, 0.01f, 0.05f);
  filterStep(f, p, 0);
  float y = 0;
  for (int i = 0; i < 200; i++) y = filterStep(f, p, 1.0f);
  CHECK_NEAR(y, 1.0f, 0.01);
}

TEST(filter_nan_passthrough) {
  ChannelFilter f; filterReset(f);
  auto p = P(FilterKind::MEDIAN, 5);
  CHECK_NEAR(filterStep(f, p, 1.0f), 1.0f, 1e-6);
  CHECK(std::isnan(filterStep(f, p, NAN)));
  CHECK_NEAR(filterStep(f, p, 1.0f), 1.0f, 1e-6);
}

TEST(filter_reset_on_param_change) {
  ChannelFilter f; filterReset(f);
  auto p = P(FilterKind::EMA, 5, 0.1f);
  for (int i = 0; i < 10; i++) filterStep(f, p, 0);
  filterReset(f);
  CHECK_NEAR(filterStep(f, p, 5.0f), 5.0f, 1e-6);
}

TEST(filter_group_delay) {
  CHECK_NEAR(filterGroupDelayMs(P(FilterKind::MOVING_AVG, 5), 10), 20, 1e-4);
  CHECK_NEAR(filterGroupDelayMs(P(FilterKind::MEDIAN, 5), 10), 20, 1e-4);
  CHECK_NEAR(filterGroupDelayMs(P(FilterKind::EMA, 5, 0.5f), 10), 10, 1e-4);
  CHECK_NEAR(filterGroupDelayMs(P(FilterKind::NONE), 10), 0, 1e-6);
  const double q = 0.01, r = 0.05, pm = (q + std::sqrt(q * q + 4 * q * r)) / 2, k = pm / (pm + r);
  CHECK_NEAR(filterGroupDelayMs(P(FilterKind::KALMAN, 5, 0.3f, 0.01f, 0.05f), 10), (1 - k) / k * 10, 0.5);
}

#include <cstring>
TEST(filter_params_equal_ignores_padding) {
  FilterParams a, b;
  std::memset(&a, 0x00, sizeof(a)); std::memset(&b, 0xAB, sizeof(b));   // different padding bytes
  a.kind = b.kind = FilterKind::MEDIAN; a.window = b.window = 5;
  a.alpha = b.alpha = 0.3f; a.q = b.q = 0.01f; a.r = b.r = 0.05f;
  CHECK(filterParamsEqual(a, b));
  b.window = 7;
  CHECK(!filterParamsEqual(a, b));
  b.window = 5; b.alpha = 0.31f;
  CHECK(!filterParamsEqual(a, b));
}
