#include <gtest/gtest.h>

#include "mca/benchmark/metrics.hpp"

TEST(BenchmarkMetrics, TrackingAccumulator) {
  mca::benchmark::TrackingAccumulator accumulator;
  accumulator.add(1.0, 0.0, 2.0);
  accumulator.add(1.0, 0.5, 1.0);

  const auto result = accumulator.result();
  EXPECT_EQ(result.samples, 2U);
  EXPECT_NEAR(result.rmse, std::sqrt(0.625), 1e-12);
  EXPECT_DOUBLE_EQ(result.max_abs_error, 1.0);
  EXPECT_NEAR(result.control_effort_rms, std::sqrt(2.5), 1e-12);
}

TEST(BenchmarkMetrics, Reset) {
  mca::benchmark::TrackingAccumulator accumulator;
  accumulator.add(1.0, 0.0, 1.0);
  accumulator.reset();
  EXPECT_EQ(accumulator.result().samples, 0U);
}
