#include "mca/benchmark/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mca::benchmark {

void TrackingAccumulator::reset() noexcept {
  squared_error_sum_ = 0.0;
  max_abs_error_ = 0.0;
  control_squared_sum_ = 0.0;
  samples_ = 0;
}

void TrackingAccumulator::add(double reference, double actual, double control) {
  if (!std::isfinite(reference) || !std::isfinite(actual) ||
      !std::isfinite(control)) {
    return;
  }

  const double error = reference - actual;
  if (!std::isfinite(error)) return;

  const double error_square = error * error;
  const double control_square = control * control;
  if (!std::isfinite(error_square) || !std::isfinite(control_square)) {
    return;
  }

  const double next_squared_error_sum = squared_error_sum_ + error_square;
  const double next_control_squared_sum =
      control_squared_sum_ + control_square;
  if (!std::isfinite(next_squared_error_sum) ||
      !std::isfinite(next_control_squared_sum)) {
    return;
  }

  const double abs_error = std::abs(error);
  if (!std::isfinite(abs_error)) return;

  // Invalid/non-finite samples are ignored rather than poisoning the
  // accumulator. samples_ counts only samples that can be represented by the
  // finite sum-of-squares state used by this lightweight benchmark metric.
  squared_error_sum_ = next_squared_error_sum;
  control_squared_sum_ = next_control_squared_sum;
  max_abs_error_ = std::max(max_abs_error_, abs_error);
  ++samples_;
}

TrackingMetrics TrackingAccumulator::result() const noexcept {
  TrackingMetrics result{};
  result.samples = samples_;
  if (samples_ == 0U) return result;
  result.rmse = std::sqrt(squared_error_sum_ / static_cast<double>(samples_));
  result.max_abs_error = max_abs_error_;
  result.control_effort_rms =
      std::sqrt(control_squared_sum_ / static_cast<double>(samples_));
  return result;
}

bool StepResponseAccumulator::configure(double initial_reference,
                                        double target_reference,
                                        double settling_tolerance_ratio,
                                        double settling_hold_time_s) noexcept {
  if (!std::isfinite(initial_reference) || !std::isfinite(target_reference) ||
      !std::isfinite(settling_tolerance_ratio) ||
      !std::isfinite(settling_hold_time_s) ||
      settling_tolerance_ratio <= 0.0 || settling_tolerance_ratio >= 1.0 ||
      settling_hold_time_s < 0.0) {
    return false;
  }

  // Validate derived values before mutating the active configuration. Finite
  // endpoints can still produce an infinite difference when subtracted.
  const double amplitude = target_reference - initial_reference;
  if (!std::isfinite(amplitude) ||
      std::abs(amplitude) <= std::numeric_limits<double>::epsilon()) {
    return false;
  }
  const double step_amplitude = std::abs(amplitude);
  const double tolerance = settling_tolerance_ratio * step_amplitude;
  if (!std::isfinite(tolerance)) return false;

  initial_reference_ = initial_reference;
  target_reference_ = target_reference;
  step_amplitude_ = step_amplitude;
  tolerance_ = tolerance;
  hold_time_s_ = settling_hold_time_s;
  configured_ = true;
  reset();
  return true;
}

void StepResponseAccumulator::reset() noexcept {
  peak_overshoot_ = 0.0;
  candidate_settling_time_s_ = std::numeric_limits<double>::quiet_NaN();
  settling_time_s_ = std::numeric_limits<double>::quiet_NaN();
}

void StepResponseAccumulator::add(double time_s, double actual) noexcept {
  if (!configured_ || !std::isfinite(time_s) || !std::isfinite(actual) ||
      time_s < 0.0) {
    return;
  }

  const bool rising = target_reference_ > initial_reference_;
  const double directional_overshoot =
      rising ? actual - target_reference_ : target_reference_ - actual;
  const double target_error = target_reference_ - actual;
  if (!std::isfinite(directional_overshoot) || !std::isfinite(target_error)) {
    return;
  }

  const double normalized_overshoot =
      std::max(0.0, directional_overshoot) / step_amplitude_;
  if (!std::isfinite(normalized_overshoot)) return;

  // Validate the time delta before touching settling state. Very large but
  // finite timestamps can otherwise overflow the subtraction and make a
  // valid candidate appear to have satisfied the hold time.
  double candidate_duration = 0.0;
  if (std::isfinite(candidate_settling_time_s_)) {
    candidate_duration = time_s - candidate_settling_time_s_;
    if (!std::isfinite(candidate_duration)) return;
  }

  peak_overshoot_ = std::max(peak_overshoot_, normalized_overshoot);

  if (std::abs(target_error) <= tolerance_) {
    if (!std::isfinite(candidate_settling_time_s_)) {
      candidate_settling_time_s_ = time_s;
      candidate_duration = 0.0;
    }
    if (candidate_duration >= hold_time_s_) {
      settling_time_s_ = candidate_settling_time_s_;
    }
  } else {
    candidate_settling_time_s_ = std::numeric_limits<double>::quiet_NaN();
    settling_time_s_ = std::numeric_limits<double>::quiet_NaN();
  }
}

double StepResponseAccumulator::overshoot() const noexcept {
  return peak_overshoot_;
}

double StepResponseAccumulator::settling_time_s() const noexcept {
  return settling_time_s_;
}

}  // namespace mca::benchmark