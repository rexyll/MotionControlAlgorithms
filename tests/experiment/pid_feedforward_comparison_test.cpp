#include <cmath>

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
