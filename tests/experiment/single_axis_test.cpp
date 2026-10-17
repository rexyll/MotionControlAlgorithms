#include <cmath>

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
