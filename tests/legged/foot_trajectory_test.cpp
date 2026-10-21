#include "mca/legged/foot_trajectory.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using mca::geometry::Vec3;
using mca::legged::FootTrajectoryParameters;
using mca::legged::FootTrajectorySample;
using mca::legged::evaluate_foot_trajectory;

TEST(FootTrajectory, MatchesEndpointsWithZeroVelocityAndAcceleration) {
  FootTrajectoryParameters p;
  p.start_position_world = {0.0, 0.0, 0.2};
  p.finish_position_world = {0.4, -0.2, 0.2};
  p.swing_height = 0.1;
  p.duration_s = 0.8;
  FootTrajectorySample start, finish;
  ASSERT_TRUE(evaluate_foot_trajectory(p, 0.0, start));
  ASSERT_TRUE(evaluate_foot_trajectory(p, p.duration_s, finish));
  EXPECT_DOUBLE_EQ(start.position_world.x, 0.0);
  EXPECT_DOUBLE_EQ(start.position_world.z, 0.2);
  EXPECT_DOUBLE_EQ(finish.position_world.x, 0.4);
  EXPECT_DOUBLE_EQ(finish.position_world.y, -0.2);
  EXPECT_DOUBLE_EQ(finish.position_world.z, 0.2);
  EXPECT_DOUBLE_EQ(start.velocity_world.x, 0.0);
  EXPECT_DOUBLE_EQ(start.velocity_world.z, 0.0);
  EXPECT_DOUBLE_EQ(start.acceleration_world.x, 0.0);
  EXPECT_DOUBLE_EQ(finish.velocity_world.y, 0.0);
  EXPECT_DOUBLE_EQ(finish.acceleration_world.z, 0.0);
}

TEST(FootTrajectory, MidSwingReachesConfiguredClearanceAndSmoothInterpolation) {
  FootTrajectoryParameters p;
  p.start_position_world = {0.0, 0.0, 0.1};
  p.finish_position_world = {1.0, 0.0, 0.1};
  p.swing_height = 0.12;
  p.duration_s = 2.0;
  FootTrajectorySample sample;
  ASSERT_TRUE(evaluate_foot_trajectory(p, 1.0, sample));
  EXPECT_NEAR(sample.position_world.x, 0.5, 1e-12);
  EXPECT_NEAR(sample.position_world.z, 0.22, 1e-12);
  EXPECT_NEAR(sample.velocity_world.z, 0.0, 1e-12);
  EXPECT_NEAR(sample.acceleration_world.x, 0.0, 1e-12);
  EXPECT_GT(sample.velocity_world.x, 0.0);
}

TEST(FootTrajectory, ClampsOutsideSwingIntervalWithStationaryEndpoints) {
  FootTrajectoryParameters p;
  p.start_position_world = {-0.2, 0.1, 0.3};
  p.finish_position_world = {0.2, -0.1, 0.3};
  FootTrajectorySample before, after;
  ASSERT_TRUE(evaluate_foot_trajectory(p, -1.0, before));
  ASSERT_TRUE(evaluate_foot_trajectory(p, 3.0, after));
  EXPECT_DOUBLE_EQ(before.position_world.x, p.start_position_world.x);
  EXPECT_DOUBLE_EQ(after.position_world.y, p.finish_position_world.y);
  EXPECT_DOUBLE_EQ(before.velocity_world.x, 0.0);
  EXPECT_DOUBLE_EQ(after.acceleration_world.z, 0.0);
}

TEST(FootTrajectory, InvalidInputsPreserveOutput) {
  FootTrajectoryParameters p;
  p.duration_s = 0.0;
  FootTrajectorySample output;
  output.position_world = {7.0, 8.0, 9.0};
  output.velocity_world = {1.0, 2.0, 3.0};
  std::string error;
  EXPECT_FALSE(evaluate_foot_trajectory(p, 0.5, output, &error));
  EXPECT_EQ(error, "swing height must be non-negative and duration positive");
  EXPECT_DOUBLE_EQ(output.position_world.x, 7.0);
  EXPECT_DOUBLE_EQ(output.velocity_world.y, 2.0);

  p.duration_s = 1.0;
  p.swing_height = -0.1;
  EXPECT_FALSE(evaluate_foot_trajectory(p, 0.5, output, &error));
  p.swing_height = 0.0;
  EXPECT_FALSE(evaluate_foot_trajectory(
      p, std::numeric_limits<double>::quiet_NaN(), output, &error));
  EXPECT_DOUBLE_EQ(output.position_world.z, 9.0);
}

TEST(FootTrajectory, ZeroClearanceAndVerticalMotionAreSupported) {
  FootTrajectoryParameters p;
  p.start_position_world = {0.0, 0.0, 0.0};
  p.finish_position_world = {0.0, 0.0, 0.3};
  p.swing_height = 0.0;
  FootTrajectorySample sample;
  ASSERT_TRUE(evaluate_foot_trajectory(p, 0.5, sample));
  EXPECT_NEAR(sample.position_world.z, 0.15, 1e-12);
  EXPECT_TRUE(std::isfinite(sample.velocity_world.z));
}
}  // namespace
