#pragma once

#include <cstddef>
#include <limits>

namespace mca::benchmark {

struct TrackingMetrics {
  double rmse{0.0};
  double max_abs_error{0.0};
  double overshoot{0.0};
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

}  // namespace mca::benchmark
