#include <cmath>

#include <gtest/gtest.h>

#include "mca/trajectory/scalar.hpp"

TEST(ScalarTrajectory, StepHasConstantTarget) {
  const mca::trajectory::StepTrajectory trajectory(2.0, -1.0);
  EXPECT_DOUBLE_EQ(trajectory.sample(-0.1).position, -1.0);
  EXPECT_DOUBLE_EQ(trajectory.sample(0.0).position, 2.0);
  EXPECT_DOUBLE_EQ(trajectory.sample(1.0).velocity, 0.0);
}

TEST(ScalarTrajectory, RampClampsAtEndpoints) {
  const mca::trajectory::RampTrajectory trajectory(0.0, 2.0, 4.0);
  EXPECT_DOUBLE_EQ(trajectory.sample(-1.0).position, 0.0);
  EXPECT_DOUBLE_EQ(trajectory.sample(2.0).position, 1.0);
  EXPECT_DOUBLE_EQ(trajectory.sample(2.0).velocity, 0.5);
  EXPECT_DOUBLE_EQ(trajectory.sample(5.0).position, 2.0);
  EXPECT_DOUBLE_EQ(trajectory.sample(5.0).velocity, 0.0);
}

TEST(ScalarTrajectory, SineProvidesPositionVelocityAcceleration) {
  constexpr double kPi = 3.14159265358979323846;
  const mca::trajectory::SineTrajectory trajectory(2.0, 1.0);
  const auto sample = trajectory.sample(0.25);
  EXPECT_NEAR(sample.position, 2.0, 1e-12);
  EXPECT_NEAR(sample.velocity, 0.0, 1e-11);
  EXPECT_NEAR(sample.acceleration, -2.0 * 4.0 * kPi * kPi, 1e-10);
}
