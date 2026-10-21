#include <cmath>
#include <gtest/gtest.h>

#include "mca/trajectory/s_curve.hpp"

namespace {
using mca::trajectory::SCurveTrajectory;

TEST(SCurveTrajectory, RestToRestReachesBothEndpoints) {
  const SCurveTrajectory trajectory(0.0, 2.0, 1.0, 2.0, 10.0);
  ASSERT_TRUE(trajectory.valid());
  ASSERT_GT(trajectory.duration(), 0.0);
  const auto start = trajectory.sample(0.0);
  const auto finish = trajectory.sample(trajectory.duration());
  EXPECT_DOUBLE_EQ(start.position, 0.0);
  EXPECT_DOUBLE_EQ(start.velocity, 0.0);
  EXPECT_DOUBLE_EQ(start.acceleration, 0.0);
  EXPECT_DOUBLE_EQ(finish.position, 2.0);
  EXPECT_DOUBLE_EQ(finish.velocity, 0.0);
  EXPECT_DOUBLE_EQ(finish.acceleration, 0.0);
}

TEST(SCurveTrajectory, RespectsVelocityAccelerationAndJerkLimits) {
  constexpr double kV = 1.2;
  constexpr double kA = 0.8;
  constexpr double kJ = 2.0;
  const SCurveTrajectory trajectory(0.0, 10.0, kV, kA, kJ);
  ASSERT_TRUE(trajectory.valid());
  const int samples = 20000;
  double previous_acceleration = trajectory.sample(0.0).acceleration;
  for (int i = 1; i < samples; ++i) {
    const double t = trajectory.duration() * static_cast<double>(i) /
                     static_cast<double>(samples);
    const auto sample = trajectory.sample(t);
    EXPECT_LE(std::abs(sample.velocity), kV + 1e-9);
    EXPECT_LE(std::abs(sample.acceleration), kA + 1e-9);
    const double dt = trajectory.duration() / static_cast<double>(samples);
    EXPECT_LE(std::abs(sample.acceleration - previous_acceleration), kJ * dt + 1e-8);
    previous_acceleration = sample.acceleration;
  }
}

TEST(SCurveTrajectory, SupportsReverseMotion) {
  const SCurveTrajectory trajectory(3.0, -1.0, 1.0, 1.5, 4.0);
  ASSERT_TRUE(trajectory.valid());
  EXPECT_NEAR(trajectory.sample(trajectory.duration() * 0.5).velocity, -1.0, 1e-9);
  EXPECT_DOUBLE_EQ(trajectory.sample(trajectory.duration()).position, -1.0);
}

TEST(SCurveTrajectory, HandlesShortMoveWithoutCruiseAndLongMoveWithCruise) {
  const SCurveTrajectory short_move(0.0, 0.01, 3.0, 2.0, 10.0);
  const SCurveTrajectory long_move(0.0, 100.0, 1.0, 1.0, 2.0);
  ASSERT_TRUE(short_move.valid());
  ASSERT_TRUE(long_move.valid());
  EXPECT_LT(short_move.duration(), long_move.duration());
  EXPECT_NEAR(short_move.sample(short_move.duration() * 0.5).position, 0.005, 1e-8);
  EXPECT_NEAR(long_move.sample(long_move.duration() * 0.5).position, 50.0, 1e-7);
}

TEST(SCurveTrajectory, RejectsInvalidLimitsAndHandlesZeroDistance) {
  const SCurveTrajectory invalid(0.0, 1.0, 0.0, 1.0, 1.0);
  EXPECT_FALSE(invalid.valid());
  EXPECT_DOUBLE_EQ(invalid.sample(0.5).position, 0.0);
  const SCurveTrajectory no_move(4.0, 4.0, 1.0, 1.0, 1.0);
  ASSERT_TRUE(no_move.valid());
  EXPECT_DOUBLE_EQ(no_move.duration(), 0.0);
  EXPECT_DOUBLE_EQ(no_move.sample(10.0).position, 4.0);
}
}  // namespace
