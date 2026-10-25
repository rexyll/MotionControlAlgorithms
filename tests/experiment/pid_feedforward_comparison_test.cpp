#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mca/experiment/pid_feedforward_comparison.hpp"

namespace {

mca::experiment::PidFeedforwardComparisonConfig make_config() {
  mca::experiment::PidFeedforwardComparisonConfig config;
  config.dt = 0.001;
  config.steps = 2000;
  config.ramp_target = 1.0;
  config.ramp_duration_s = 1.0;
  config.pid.kp = 10.0;
  config.pid.ki = 0.0;
  config.pid.kd = 4.0;
  config.pid.output_min = -100.0;
  config.pid.output_max = 100.0;
  config.pid.integral_min = -10.0;
  config.pid.integral_max = 10.0;
  config.feedforward.velocity_gain = 0.5;
  config.feedforward.acceleration_gain = 1.0;
  config.plant = mca::plant::SecondOrderPlant(1.0, 0.5, 0.0);
  return config;
}

}  // namespace

TEST(PidFeedforwardComparison, RejectsInvalidConfiguration) {
  mca::experiment::PidFeedforwardComparison runner;
  auto config = make_config();
  config.dt = 0.0;
  EXPECT_FALSE(runner.configure(config).ok());
}

TEST(PidFeedforwardComparison, QuantifiesFeedforwardBenefit) {
  mca::experiment::PidFeedforwardComparison runner;
  ASSERT_TRUE(runner.configure(make_config()).ok());

  mca::experiment::PidFeedforwardComparisonResult result;
  ASSERT_TRUE(runner.run(result).ok());

  EXPECT_EQ(result.pid_only.tracking.samples, 2000U);
  EXPECT_EQ(result.pid_plus_feedforward.tracking.samples, 2000U);
  EXPECT_LT(result.pid_plus_feedforward.tracking.rmse,
            result.pid_only.tracking.rmse * 0.95);
  EXPECT_GT(result.rmse_reduction_fraction, 0.05);
  EXPECT_LT(std::abs(result.pid_plus_feedforward.final_position - 1.0), 0.05);
  EXPECT_LT(std::abs(result.pid_plus_feedforward.final_velocity), 0.1);
  EXPECT_TRUE(std::isfinite(result.pid_only.tracking.overshoot));
  EXPECT_TRUE(std::isfinite(result.pid_plus_feedforward.tracking.overshoot));
}

TEST(PidFeedforwardComparison, SameReferenceAndPlantAreUsed) {
  mca::experiment::PidFeedforwardComparison runner;
  auto config = make_config();
  config.feedforward = {};
  ASSERT_TRUE(runner.configure(config).ok());

  mca::experiment::PidFeedforwardComparisonResult result;
  ASSERT_TRUE(runner.run(result).ok());

  EXPECT_NEAR(result.rmse_reduction_fraction, 0.0, 1e-12);
  EXPECT_NEAR(result.pid_plus_feedforward.final_position,
              result.pid_only.final_position, 1e-12);
  EXPECT_NEAR(result.pid_plus_feedforward.final_velocity,
              result.pid_only.final_velocity, 1e-12);
  EXPECT_NEAR(result.pid_plus_feedforward.tracking.rmse,
              result.pid_only.tracking.rmse, 1e-12);
}


TEST(PidFeedforwardComparison, InvalidReconfigurationPreservesWorkingRunner) {
  mca::experiment::PidFeedforwardComparison runner;
  const auto valid = make_config();
  ASSERT_TRUE(runner.configure(valid).ok());

  mca::experiment::PidFeedforwardComparisonResult baseline;
  ASSERT_TRUE(runner.run(baseline).ok());

  auto invalid = valid;
  invalid.dt = 0.0;
  EXPECT_FALSE(runner.configure(invalid).ok());
  EXPECT_TRUE(runner.configured());

  mca::experiment::PidFeedforwardComparisonResult after_invalid;
  ASSERT_TRUE(runner.run(after_invalid).ok());
  EXPECT_DOUBLE_EQ(after_invalid.pid_only.final_position,
                   baseline.pid_only.final_position);
  EXPECT_DOUBLE_EQ(after_invalid.pid_plus_feedforward.final_position,
                   baseline.pid_plus_feedforward.final_position);
  EXPECT_DOUBLE_EQ(after_invalid.pid_only.tracking.rmse,
                   baseline.pid_only.tracking.rmse);
  EXPECT_DOUBLE_EQ(after_invalid.rmse_reduction_fraction,
                   baseline.rmse_reduction_fraction);
}

TEST(PidFeedforwardComparison, FailedRunPreservesCallerResultAndRunner) {
  mca::experiment::PidFeedforwardComparison runner;
  auto config = make_config();
  config.dt = 1.0e308;
  config.steps = 1;
  config.ramp_duration_s = 1.0;
  config.feedforward.velocity_gain = std::numeric_limits<double>::max();
  config.feedforward.acceleration_gain = 0.0;
  config.feedforward.constant_bias = std::numeric_limits<double>::max();
  ASSERT_TRUE(runner.configure(config).ok());

  mca::experiment::PidFeedforwardComparisonResult result;
  result.pid_only.final_position = 123.0;
  result.pid_plus_feedforward.final_velocity = 456.0;
  result.rmse_reduction_fraction = 7.0;

  EXPECT_FALSE(runner.run(result).ok());
  EXPECT_TRUE(runner.configured());
  EXPECT_DOUBLE_EQ(result.pid_only.final_position, 123.0);
  EXPECT_DOUBLE_EQ(result.pid_plus_feedforward.final_velocity, 456.0);
  EXPECT_DOUBLE_EQ(result.rmse_reduction_fraction, 7.0);

  config.dt = 0.001;
  config.steps = 100;
  config.feedforward.velocity_gain = 1.0;
  config.feedforward.constant_bias = 0.0;
  ASSERT_TRUE(runner.configure(config).ok());
  mca::experiment::PidFeedforwardComparisonResult recovered;
  ASSERT_TRUE(runner.run(recovered).ok());
  EXPECT_EQ(recovered.pid_only.tracking.samples, 100U);
  EXPECT_EQ(recovered.pid_plus_feedforward.tracking.samples, 100U);
}


TEST(PidFeedforwardComparison, RejectsOverflowingTimeHorizonTransactionally) {
  mca::experiment::PidFeedforwardComparison runner;
  const auto valid = make_config();
  ASSERT_TRUE(runner.configure(valid).ok());

  mca::experiment::PidFeedforwardComparisonResult baseline;
  ASSERT_TRUE(runner.run(baseline).ok());

  auto invalid = valid;
  invalid.dt = std::numeric_limits<double>::max();
  invalid.steps = 2;
  const auto status = runner.configure(invalid);
  EXPECT_EQ(status.code(), mca::core::StatusCode::NumericalError);
  EXPECT_TRUE(runner.configured());

  mca::experiment::PidFeedforwardComparisonResult after_invalid;
  ASSERT_TRUE(runner.run(after_invalid).ok());
  EXPECT_DOUBLE_EQ(after_invalid.pid_only.final_position,
                   baseline.pid_only.final_position);
  EXPECT_DOUBLE_EQ(after_invalid.pid_plus_feedforward.final_position,
                   baseline.pid_plus_feedforward.final_position);
  EXPECT_DOUBLE_EQ(after_invalid.pid_only.tracking.rmse,
                   baseline.pid_only.tracking.rmse);
}

TEST(PidFeedforwardComparison, UnconfiguredPlantReconfigurationPreservesRunner) {
  mca::experiment::PidFeedforwardComparison runner;
  const auto valid = make_config();
  ASSERT_TRUE(runner.configure(valid).ok());

  mca::experiment::PidFeedforwardComparisonResult baseline;
  ASSERT_TRUE(runner.run(baseline).ok());

  auto invalid = valid;
  invalid.plant = mca::plant::SecondOrderPlant{};
  EXPECT_FALSE(runner.configure(invalid).ok());
  EXPECT_TRUE(runner.configured());

  mca::experiment::PidFeedforwardComparisonResult after_invalid;
  ASSERT_TRUE(runner.run(after_invalid).ok());
  EXPECT_DOUBLE_EQ(after_invalid.pid_only.final_position,
                   baseline.pid_only.final_position);
  EXPECT_DOUBLE_EQ(after_invalid.pid_plus_feedforward.final_position,
                   baseline.pid_plus_feedforward.final_position);
  EXPECT_DOUBLE_EQ(after_invalid.pid_only.tracking.rmse,
                   baseline.pid_only.tracking.rmse);
}
