#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "mca/path_tracking/mpc_path_tracker.hpp"

namespace {
using mca::path_tracking::MpcCommand;
using mca::path_tracking::MpcConfig;
using mca::path_tracking::MpcPathPoint2D;
using mca::path_tracking::MpcPathTracker;
using mca::path_tracking::MpcPose2D;

TEST(MpcPathTracker, RejectsInvalidConfiguration) {
  MpcPathTracker tracker;
  MpcConfig config;
  config.horizon = 0U;
  EXPECT_FALSE(tracker.configure(config));
  config = MpcConfig{};
  config.optimization_iterations = 0U;
  EXPECT_FALSE(tracker.configure(config));
  config = MpcConfig{};
  config.steering_weight = 0.0;
  EXPECT_FALSE(tracker.configure(config));
  config = MpcConfig{};
  config.gradient_step = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(tracker.configure(config));
}

TEST(MpcPathTracker, StraightPathAtCenterProducesZeroSteering) {
  MpcPathTracker tracker;
  ASSERT_TRUE(tracker.configure(MpcConfig{}));
  MpcCommand command;
  ASSERT_TRUE(tracker.compute(MpcPose2D{1.0, 0.0, 0.0},
                              {{0.0, 0.0}, {10.0, 0.0}}, command));
  EXPECT_NEAR(command.lateral_error, 0.0, 1e-12);
  EXPECT_NEAR(command.heading_error, 0.0, 1e-12);
  EXPECT_NEAR(command.steering_angle, 0.0, 1e-12);
  EXPECT_NEAR(command.angular_z, 0.0, 1e-12);
  EXPECT_GE(command.predicted_cost, 0.0);
}

TEST(MpcPathTracker, LateralAndHeadingErrorsSteerTowardPath) {
  MpcPathTracker tracker;
  ASSERT_TRUE(tracker.configure(MpcConfig{}));
  MpcCommand command;
  ASSERT_TRUE(tracker.compute(MpcPose2D{1.0, 0.5, 0.0},
                              {{0.0, 0.0}, {10.0, 0.0}}, command));
  EXPECT_GT(command.lateral_error, 0.0);
  EXPECT_LT(command.steering_angle, 0.0);
  EXPECT_LT(command.angular_z, 0.0);
  ASSERT_TRUE(tracker.compute(MpcPose2D{1.0, 0.0, 0.3},
                              {{0.0, 0.0}, {10.0, 0.0}}, command));
  EXPECT_LT(command.steering_angle, 0.0);
}

TEST(MpcPathTracker, SaturatesSteeringAndSelectsNearestSegment) {
  MpcConfig config;
  config.max_steering_angle = 0.15;
  config.horizon = 8U;
  MpcPathTracker tracker;
  ASSERT_TRUE(tracker.configure(config));
  MpcCommand command;
  ASSERT_TRUE(tracker.compute(MpcPose2D{1.1, 0.8, 1.5707963267948966},
                              {{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {1.0, 2.0}},
                              command));
  EXPECT_EQ(command.target_segment, 2U);
  EXPECT_NEAR(command.lateral_error, -0.1, 1e-12);
  EXPECT_LE(std::abs(command.steering_angle), config.max_steering_angle);
}

TEST(MpcPathTracker, InvalidInputDoesNotModifyOutput) {
  MpcPathTracker tracker;
  ASSERT_TRUE(tracker.configure(MpcConfig{}));
  MpcCommand command{1.0, 2.0, 3.0, 4.0, 5.0, 6U, 7.0};
  EXPECT_FALSE(tracker.compute(MpcPose2D{}, {{0.0, 0.0}}, command));
  EXPECT_DOUBLE_EQ(command.steering_angle, 2.0);
  EXPECT_FALSE(tracker.compute(MpcPose2D{}, {
      {0.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 0.0}}, command));
  EXPECT_DOUBLE_EQ(command.predicted_cost, 7.0);
}

TEST(MpcPathTracker, ClosedLoopStraightPathConvergesFromLateralOffset) {
  MpcConfig config;
  config.sample_time = 0.02;
  config.horizon = 12U;
  config.optimization_iterations = 100U;
  config.gradient_step = 0.002;
  MpcPathTracker tracker;
  ASSERT_TRUE(tracker.configure(config));
  std::vector<MpcPathPoint2D> path;
  for (int i = 0; i <= 400; ++i) {
    path.push_back(MpcPathPoint2D{0.1 * static_cast<double>(i), 0.0});
  }
  MpcPose2D pose{0.0, 0.4, 0.0};
  for (int step = 0; step < 1500; ++step) {
    MpcCommand command;
    ASSERT_TRUE(tracker.compute(pose, path, command));
    pose.x += command.linear_x * std::cos(pose.yaw) * config.sample_time;
    pose.y += command.linear_x * std::sin(pose.yaw) * config.sample_time;
    pose.yaw += command.angular_z * config.sample_time;
  }
  EXPECT_LT(std::abs(pose.y), 0.1);
  EXPECT_GT(pose.x, 8.0);
}

}  // namespace
