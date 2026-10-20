#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include "mca/core/types.hpp"

namespace mca::analysis {

struct StepResponseMetrics {
  // Peak overshoot as a fraction of the absolute step amplitude.
  double overshoot_fraction{0.0};
  double overshoot_percent{0.0};
  double peak_value{0.0};
  double peak_time_s{std::numeric_limits<double>::quiet_NaN()};

  // Earliest time from which the response remains inside the settling band
  // through the end of the supplied sample set and satisfies hold_time_s.
  // NaN means the response did not settle in the supplied data.
  double settling_time_s{std::numeric_limits<double>::quiet_NaN()};
  bool settled{false};
};

// Evaluate a sampled scalar step response without allocating memory.
//
// The sample arrays must contain monotonically increasing finite timestamps
// and finite response values. The step is defined by initial_value and
// target_value; overshoot is normalized by abs(target - initial). For a
// positive step, values above target are overshoot. For a negative step,
// values below target are overshoot. Settling uses
// abs(actual - target) <= tolerance_ratio * abs(target - initial), and the
// response must remain in that band for at least hold_time_s and through the
// end of the supplied samples.
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
  const double step_amplitude = std::abs(step);
  const double tolerance = tolerance_ratio * step_amplitude;

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
    const double normalized_overshoot =
        std::max(0.0, directional_overshoot) / step_amplitude;
    if (normalized_overshoot > peak_overshoot) {
      peak_overshoot = normalized_overshoot;
      peak_value = value;
      peak_time = time;
    }

    if (std::abs(value - target_value) <= tolerance) {
      if (!core::is_finite(candidate_start)) {
        candidate_start = time;
      }
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

  output.overshoot_fraction = peak_overshoot;
  output.overshoot_percent = peak_overshoot * 100.0;
  output.peak_value = peak_value;
  output.peak_time_s = peak_time;
  output.settling_time_s = settling_time;
  output.settled = settled;
  return {};
}

}  // namespace mca::analysis
