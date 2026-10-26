#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mca/experiment/single_axis.hpp"

TEST(SingleAxisExperiment, ClosedLoopReachesStepReference) {
  mca::experiment::SingleAxisExperiment experiment;
  mca::experiment::SingleAxisConfig config;
  config.dt = 0.001;
  config.steps = 10000;
  config.reference = 1.0;

  mca::plant::SecondOrderPlant plant(1.0, 4.0, 0.0);
  mca::control::PidConfig pid;
  pid.kp = 100.0;
  pid.kd = 20.0;
  pid.ki = 10.0;
  pid.output_min = -100.0;
  pid.output_max = 100.0;
  pid.integral_min = -20.0;
  pid.integral_max = 20.0;

  ASSERT_TRUE(experiment.configure(config, plant, pid).ok());

  mca::experiment::SingleAxisResult result;
  ASSERT_TRUE(experiment.run(result).ok());
  EXPECT_NEAR(result.final_position, 1.0, 0.02);
  EXPECT_NEAR(result.final_velocity, 0.0, 0.02);
  EXPECT_LT(result.metrics.rmse, 0.2);
  EXPECT_GE(result.metrics.overshoot, 0.0);
  EXPECT_TRUE(std::isfinite(result.metrics.settling_time_s));
  EXPECT_GT(result.metrics.settling_time_s, 0.0);
}

TEST(SingleAxisExperiment, RejectsInvalidTimeStep) {
  mca::experiment::SingleAxisExperiment experiment;
  mca::experiment::SingleAxisConfig config;
  config.dt = 0.0;
  mca::plant::SecondOrderPlant plant(1.0, 1.0);
  mca::control::PidConfig pid;
  pid.kp = 1.0;
  EXPECT_FALSE(experiment.configure(config, plant, pid).ok());
}

TEST(SingleAxisExperiment, FailedReconfigurationPreservesActiveExperiment) {
  mca::experiment::SingleAxisExperiment experiment;
  mca::experiment::SingleAxisConfig config;
  config.dt = 0.001;
  config.steps = 1000;
  config.reference = 1.0;

  mca::plant::SecondOrderPlant plant(1.0, 4.0, 0.0);
  mca::control::PidConfig pid;
  pid.kp = 100.0;
  pid.kd = 20.0;
  pid.ki = 10.0;
  pid.output_min = -100.0;
  pid.output_max = 100.0;
  pid.integral_min = -20.0;
  pid.integral_max = 20.0;

  ASSERT_TRUE(experiment.configure(config, plant, pid).ok());

  mca::experiment::SingleAxisResult baseline;
  ASSERT_TRUE(experiment.run(baseline).ok());

  auto invalid_config = config;
  invalid_config.dt = 0.0;
  EXPECT_FALSE(experiment.configure(invalid_config, plant, pid).ok());

  mca::experiment::SingleAxisResult after_invalid;
  ASSERT_TRUE(experiment.run(after_invalid).ok());
  EXPECT_DOUBLE_EQ(after_invalid.final_position, baseline.final_position);
  EXPECT_DOUBLE_EQ(after_invalid.final_velocity, baseline.final_velocity);
  EXPECT_DOUBLE_EQ(after_invalid.metrics.rmse, baseline.metrics.rmse);
  EXPECT_EQ(after_invalid.metrics.samples, baseline.metrics.samples);
  EXPECT_DOUBLE_EQ(after_invalid.metrics.overshoot, baseline.metrics.overshoot);
  EXPECT_DOUBLE_EQ(after_invalid.metrics.settling_time_s,
                   baseline.metrics.settling_time_s);

  mca::plant::SecondOrderPlant unconfigured_plant;
  EXPECT_FALSE(experiment.configure(config, unconfigured_plant, pid).ok());
  mca::experiment::SingleAxisResult after_invalid_plant;
  ASSERT_TRUE(experiment.run(after_invalid_plant).ok());
  EXPECT_DOUBLE_EQ(after_invalid_plant.final_position, baseline.final_position);
  EXPECT_DOUBLE_EQ(after_invalid_plant.final_velocity, baseline.final_velocity);
  EXPECT_EQ(after_invalid_plant.metrics.samples, baseline.metrics.samples);
}

TEST(SingleAxisExperiment, FailedRunPreservesStateAndResult) {
  mca::experiment::SingleAxisExperiment experiment;
  mca::experiment::SingleAxisConfig config;
  config.dt = 1.0e308;
  config.steps = 1;
  config.reference = 1.0;

  mca::plant::SecondOrderPlant plant(1.0, 0.0, 0.0);
  mca::control::PidConfig pid;
  pid.kp = 1.0;
  pid.output_min = -1.0;
  pid.output_max = 1.0;

  ASSERT_TRUE(experiment.configure(config, plant, pid).ok());

  mca::experiment::SingleAxisResult result;
  result.final_position = 123.0;
  result.final_velocity = 456.0;
  result.metrics.samples = 789U;
  result.metrics.rmse = 3.0;

  const auto before_position = experiment.run(result);
  EXPECT_FALSE(before_position.ok());
  EXPECT_DOUBLE_EQ(result.final_position, 123.0);
  EXPECT_DOUBLE_EQ(result.final_velocity, 456.0);
  EXPECT_EQ(result.metrics.samples, 789U);
  EXPECT_DOUBLE_EQ(result.metrics.rmse, 3.0);

  // The failed run must not poison the active experiment. Reconfiguration to
  // a valid deterministic setup and a subsequent run must still succeed.
  config.dt = 0.001;
  config.steps = 100;
  ASSERT_TRUE(experiment.configure(config, plant, pid).ok());
  mca::experiment::SingleAxisResult recovered;
  ASSERT_TRUE(experiment.run(recovered).ok());
  EXPECT_EQ(recovered.metrics.samples, 100U);
  EXPECT_TRUE(std::isfinite(recovered.final_position));
  EXPECT_TRUE(std::isfinite(recovered.final_velocity));
}


TEST(SingleAxisExperiment, RejectsOverflowingSimulationHorizonTransactionally) {
  mca::experiment::SingleAxisExperiment experiment;
  mca::experiment::SingleAxisConfig config;
  config.dt = 0.001;
  config.steps = 100;
  config.reference = 1.0;
  mca::plant::SecondOrderPlant plant(1.0, 4.0, 0.0);
  mca::control::PidConfig pid;
  pid.kp = 100.0;
  pid.kd = 20.0;
  pid.ki = 10.0;
  pid.output_min = -100.0;
  pid.output_max = 100.0;
  pid.integral_min = -20.0;
  pid.integral_max = 20.0;

  ASSERT_TRUE(experiment.configure(config, plant, pid).ok());
  mca::experiment::SingleAxisResult baseline;
  ASSERT_TRUE(experiment.run(baseline).ok());

  auto invalid = config;
  invalid.dt = std::numeric_limits<double>::max();
  invalid.steps = 2;
  const auto status = experiment.configure(invalid, plant, pid);
  EXPECT_EQ(status.code, mca::core::StatusCode::NumericalError);

  mca::experiment::SingleAxisResult after_invalid;
  ASSERT_TRUE(experiment.run(after_invalid).ok());
  EXPECT_DOUBLE_EQ(after_invalid.final_position, baseline.final_position);
  EXPECT_DOUBLE_EQ(after_invalid.final_velocity, baseline.final_velocity);
  EXPECT_DOUBLE_EQ(after_invalid.metrics.rmse, baseline.metrics.rmse);
  EXPECT_EQ(after_invalid.metrics.samples, config.steps);
}

TEST(SingleAxisExperiment, ReportsCompleteFiniteMetrics) {
  mca::experiment::SingleAxisExperiment experiment;
  mca::experiment::SingleAxisConfig config;
  config.dt = 0.001;
  config.steps = 37;
  config.reference = 1.0;
  mca::plant::SecondOrderPlant plant(1.0, 4.0, 0.0);
  mca::control::PidConfig pid;
  pid.kp = 100.0;
  pid.kd = 20.0;
  pid.ki = 10.0;
  pid.output_min = -100.0;
  pid.output_max = 100.0;
  pid.integral_min = -20.0;
  pid.integral_max = 20.0;

  ASSERT_TRUE(experiment.configure(config, plant, pid).ok());
  mca::experiment::SingleAxisResult result;
  ASSERT_TRUE(experiment.run(result).ok());
  EXPECT_EQ(result.metrics.samples, 37U);
  EXPECT_TRUE(std::isfinite(result.metrics.rmse));
  EXPECT_TRUE(std::isfinite(result.metrics.max_abs_error));
  EXPECT_TRUE(std::isfinite(result.metrics.overshoot));
  EXPECT_TRUE(std::isfinite(result.metrics.control_effort_rms));
  EXPECT_TRUE(std::isfinite(result.metrics.settling_time_s) ||
              std::isnan(result.metrics.settling_time_s));
  EXPECT_TRUE(std::isfinite(result.final_position));
  EXPECT_TRUE(std::isfinite(result.final_velocity));
}
