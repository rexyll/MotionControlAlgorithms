#include <cmath>
#include <limits>

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

TEST(BenchmarkMetrics, StepResponseOvershootAndSettling) {
  mca::benchmark::StepResponseAccumulator accumulator;
  ASSERT_TRUE(accumulator.configure(0.0, 1.0, 0.02, 0.1));
  accumulator.add(0.0, 0.0);
  accumulator.add(0.1, 1.1);
  accumulator.add(0.2, 1.01);
  accumulator.add(0.31, 1.0);
  EXPECT_NEAR(accumulator.overshoot(), 0.1, 1e-12);
  EXPECT_NEAR(accumulator.settling_time_s(), 0.2, 1e-12);
}

TEST(BenchmarkMetrics, SettlingIsInvalidatedByLaterExcursion) {
  mca::benchmark::StepResponseAccumulator accumulator;
  ASSERT_TRUE(accumulator.configure(0.0, 1.0, 0.02, 0.05));
  accumulator.add(0.0, 1.0);
  accumulator.add(0.06, 1.0);
  EXPECT_NEAR(accumulator.settling_time_s(), 0.0, 1e-12);
  accumulator.add(0.07, 1.1);
  EXPECT_TRUE(std::isnan(accumulator.settling_time_s()));
}

TEST(BenchmarkMetrics, RejectsInvalidStepConfiguration) {
  mca::benchmark::StepResponseAccumulator accumulator;
  EXPECT_FALSE(accumulator.configure(1.0, 1.0));
  EXPECT_FALSE(accumulator.configure(0.0, 1.0, 1.0));
}

TEST(BenchmarkMetrics, FailedStepConfigurationPreservesActiveConfiguration) {
  mca::benchmark::StepResponseAccumulator accumulator;
  ASSERT_TRUE(accumulator.configure(0.0, 1.0, 0.02, 0.05));

  EXPECT_FALSE(accumulator.configure(1.0, 1.0, 0.02, 0.05));
  accumulator.add(0.0, 1.0);
  accumulator.add(0.06, 1.0);
  EXPECT_DOUBLE_EQ(accumulator.settling_time_s(), 0.0);

  accumulator.reset();
  EXPECT_FALSE(accumulator.configure(-std::numeric_limits<double>::max(),
                                     std::numeric_limits<double>::max(),
                                     0.02, 0.05));
  accumulator.add(0.0, 1.0);
  accumulator.add(0.06, 1.0);
  EXPECT_DOUBLE_EQ(accumulator.settling_time_s(), 0.0);
}

TEST(BenchmarkMetrics, StepResponseOverflowingSamplePreservesMetrics) {
  mca::benchmark::StepResponseAccumulator accumulator;
  ASSERT_TRUE(accumulator.configure(0.0, -0.5 * std::numeric_limits<double>::max(),
                                    0.02, 0.05));
  accumulator.add(0.0, -0.5 * std::numeric_limits<double>::max());
  accumulator.add(0.06, -0.5 * std::numeric_limits<double>::max());
  ASSERT_DOUBLE_EQ(accumulator.overshoot(), 0.0);
  ASSERT_DOUBLE_EQ(accumulator.settling_time_s(), 0.0);

  // target - actual overflows here; the invalid sample must not turn the
  // persistent peak or settling state into non-finite values.
  accumulator.add(0.07, std::numeric_limits<double>::max());
  EXPECT_DOUBLE_EQ(accumulator.overshoot(), 0.0);
  EXPECT_DOUBLE_EQ(accumulator.settling_time_s(), 0.0);

  accumulator.add(0.08, -0.5 * std::numeric_limits<double>::max());
  EXPECT_DOUBLE_EQ(accumulator.overshoot(), 0.0);
  EXPECT_DOUBLE_EQ(accumulator.settling_time_s(), 0.0);
}
