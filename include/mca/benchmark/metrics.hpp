#pragma once

#include <cstddef>
#include <limits>

namespace mca::benchmark {

struct TrackingMetrics {
  double rmse{0.0};
  double max_abs_error{0.0};
  // Fraction of step amplitude (0.1 means 10% overshoot).
  double overshoot{0.0};
  // NaN means the response never stayed inside the settling band long enough.
  double settling_time_s{std::numeric_limits<double>::quiet_NaN()};
  double control_effort_rms{0.0};
  std::size_t samples{0};
};

struct TimingMetrics {
  double mean_us{0.0};
  double max_us{0.0};
  double p99_us{0.0};
  std::size_t deadline_misses{0};
};

struct ConstraintMetrics {
  std::size_t violations{0};
  double max_violation{0.0};
};

struct BenchmarkResult {
  TrackingMetrics tracking{};
  TimingMetrics timing{};
  ConstraintMetrics constraints{};
};

class TrackingAccumulator {
 public:
  void reset() noexcept;
  void add(double reference, double actual, double control);
  TrackingMetrics result() const noexcept;

 private:
  double squared_error_sum_{0.0};
  double max_abs_error_{0.0};
  double control_squared_sum_{0.0};
  std::size_t samples_{0};
};

// Step-response-only metrics. Overshoot is normalized by step amplitude.
// Settling requires the response to remain inside the configured band for
// settling_hold_time_s; a later excursion invalidates the earlier candidate.
class StepResponseAccumulator {
 public:
  bool configure(double initial_reference, double target_reference,
                 double settling_tolerance_ratio = 0.02,
                 double settling_hold_time_s = 0.05) noexcept;
  void reset() noexcept;
  void add(double time_s, double actual) noexcept;
  double overshoot() const noexcept;
  double settling_time_s() const noexcept;

 private:
  double initial_reference_{0.0};
  double target_reference_{0.0};
  double step_amplitude_{0.0};
  double tolerance_{0.0};
  double hold_time_s_{0.0};
  double peak_overshoot_{0.0};
  double candidate_settling_time_s_{std::numeric_limits<double>::quiet_NaN()};
  double settling_time_s_{std::numeric_limits<double>::quiet_NaN()};
  bool configured_{false};
};

}  // namespace mca::benchmark
