#include "mca/benchmark/metrics.hpp"

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
  const double error = reference - actual;
  squared_error_sum_ += error * error;
  control_squared_sum_ += control * control;
  max_abs_error_ = std::max(max_abs_error_, std::abs(error));
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

}  // namespace mca::benchmark
