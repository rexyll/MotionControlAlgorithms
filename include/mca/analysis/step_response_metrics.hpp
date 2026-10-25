#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include "mca/core/types.hpp"

namespace mca::analysis {

struct StepResponseMetrics {
  double overshoot_fraction{0.0};
  double overshoot_percent{0.0};
  double peak_value{0.0};
  double peak_time_s{std::numeric_limits<double>::quiet_NaN()};
  double settling_time_s{std::numeric_limits<double>::quiet_NaN()};
  bool settled{false};
};

// Evaluate a sampled scalar step response without allocating memory.
// Settling is the earliest sample time from which every subsequent sample
// remains inside the tolerance band through the end of the supplied data,
// with at least hold_time_s elapsed. Overshoot is normalized by step amplitude.
inline core::Status compute_step_response_metrics(
    const double* times_s, const double* values, std::size_t sample_count,
    double initial_value, double target_value,
    double tolerance_ratio, double hold_time_s,
    StepResponseMetrics& output) noexcept {
  output = {};

  if (sample_count == 0U || times_s == nullptr || values == nullptr ||
      !core::is_finite(initial_value) || !core::is_finite(target_value) ||
      !core::is_finite(tolerance_ratio) || !core::is_finite(hold_time_s) ||
      tolerance_ratio <= 0.0 || tolerance_ratio >= 1.0 || hold_time_s < 0.0 ||
      std::abs(target_value - initial_value) <= core::kEpsilon) {
    return {core::StatusCode::InvalidArgument};
  }

  const double step = target_value - initial_value;
  if (!core::is_finite(step)) {
    return {core::StatusCode::InvalidArgument};
  }
  const double step_amplitude = std::abs(step);
  const double tolerance = tolerance_ratio * step_amplitude;
  const double tolerance_margin =
      32.0 * std::numeric_limits<double>::epsilon() *
      std::max({1.0, std::abs(target_value), step_amplitude});
  const double settling_limit = tolerance + tolerance_margin;
  if (!core::is_finite(step_amplitude) || !core::is_finite(tolerance) ||
      !core::is_finite(tolerance_margin) || !core::is_finite(settling_limit)) {
    return {core::StatusCode::InvalidArgument};
  }

  double peak_overshoot = 0.0;
  double peak_value = values[0];
  double peak_time = times_s[0];
  double candidate_start = std::numeric_limits<double>::quiet_NaN();
  double settling_time = std::numeric_limits<double>::quiet_NaN();
  bool settled = false;

  for (std::size_t i = 0U; i < sample_count; ++i) {
    const double time = times_s[i];
    const double value = values[i];
    if (!core::is_finite(time) || !core::is_finite(value) || time < 0.0 ||
        (i > 0U && !(time > times_s[i - 1U]))) {
      return {core::StatusCode::InvalidArgument};
    }

    const double directional_overshoot =
        step > 0.0 ? value - target_value : target_value - value;
    const double target_error = value - target_value;
    if (!core::is_finite(directional_overshoot) ||
        !core::is_finite(target_error)) {
      return {core::StatusCode::InvalidArgument};
    }
    const double normalized_overshoot =
        std::max(0.0, directional_overshoot) / step_amplitude;
    if (!core::is_finite(normalized_overshoot)) {
      return {core::StatusCode::InvalidArgument};
    }
    if (normalized_overshoot > peak_overshoot) {
      peak_overshoot = normalized_overshoot;
      peak_value = value;
      peak_time = time;
    }

    if (std::abs(target_error) <= settling_limit) {
      if (!core::is_finite(candidate_start)) candidate_start = time;
      if (time - candidate_start >= hold_time_s) {
        settling_time = candidate_start;
        settled = true;
      }
    } else {
      candidate_start = std::numeric_limits<double>::quiet_NaN();
      settling_time = std::numeric_limits<double>::quiet_NaN();
      settled = false;
    }
  }

  const double overshoot_percent = peak_overshoot * 100.0;
  if (!core::is_finite(overshoot_percent)) {
    return {core::StatusCode::InvalidArgument};
  }

  output.overshoot_fraction = peak_overshoot;
  output.overshoot_percent = overshoot_percent;
  output.peak_value = peak_value;
  output.peak_time_s = peak_time;
  output.settling_time_s = settling_time;
  output.settled = settled;
  return {};
}

}  // namespace mca::analysis
