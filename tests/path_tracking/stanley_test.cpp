#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "mca/path_tracking/stanley.hpp"

namespace {
using mca::path_tracking::Stanley;
using mca::path_tracking::StanleyCommand;
using mca::path_tracking::StanleyConfig;
using mca::path_tracking::StanleyPathPoint2D;
using mca::path_tracking::StanleyPose2D;

TEST(Stanley, RejectsInvalidConfiguration) {
  Stanley controller;
  EXPECT_FALSE(controller.configure(StanleyConfig{1.0, 1.0, 0.0, 1.0, 0.5, 0.6}));
  EXPECT_FALSE(controller.configure(StanleyConfig{1.0, 1.0, 0.1, 1.0, 0.0, 0.6}));
  EXPECT_FALSE(controller.configure(StanleyConfig{
      std::numeric_limits<double>::quiet_NaN(), 1.0, 0.1, 1.0, 0.5, 0.6}));
}

TEST(Stanley, StraightPathAtCenterProducesNoCorrection) {
  Stanley controller;
  ASSERT_TRUE(controller.configure(StanleyConfig{1.0, 1.0, 0.1, 1.0, 0.5, 0.6}));
  const std::vector<StanleyPathPoint2D> path{{0.0, 0.0}, {5.0, 0.0}};
  StanleyCommand command;
  ASSERT_TRUE(controller.compute(StanleyPose2D{1.0, 0.0, 0.0}, path, command));
  EXPECT_NEAR(command.cross_track_error, 0.0, 1e-12);
  EXPECT_NEAR(command.heading_error, 0.0, 1e-12);
  EXPECT_NEAR(command.steering_angle, 0.0, 1e-12);
  EXPECT_NEAR(command.angular_z, 0.0, 1e-12);
}

TEST(Stanley, LateralOffsetSteersTowardPath) {
  Stanley controller;
  ASSERT_TRUE(controller.configure(StanleyConfig{2.0, 1.0, 0.1, 1.0, 0.5, 0.6}));
  const std::vector<StanleyPathPoint2D> path{{0.0, 0.0}, {10.0, 0.0}};
  StanleyCommand command;
  ASSERT_TRUE(controller.compute(StanleyPose2D{1.0, 0.5, 0.0}, path, command));
  EXPECT_LT(command.cross_track_error, 0.0);
  EXPECT_LT(command.steering_angle, 0.0);
  EXPECT_LT(command.angular_z, 0.0);
}

TEST(Stanley, CorrectsHeadingErrorAndClampsSteering) {
  Stanley controller;
  ASSERT_TRUE(controller.configure(StanleyConfig{1.0, 2.0, 0.1, 1.0, 0.5, 0.25}));
  const std::vector<StanleyPathPoint2D> path{{0.0, 0.0}, {10.0, 0.0}};
  StanleyCommand command;
  ASSERT_TRUE(controller.compute(StanleyPose2D{1.0, 0.0, 0.5}, path, command));
  EXPECT_NEAR(command.heading_error, -0.5, 1e-12);
  EXPECT_NEAR(command.steering_angle, -0.25, 1e-12);
}

TEST(Stanley, ChoosesNearestSegmentAndHandlesDuplicateVertices) {
  Stanley controller;
  ASSERT_TRUE(controller.configure(StanleyConfig{1.0, 1.0, 0.1, 1.0, 0.5, 0.6}));
  const std::vector<StanleyPathPoint2D> path{
      {0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {1.0, 2.0}};
  StanleyCommand command;
  ASSERT_TRUE(controller.compute(StanleyPose2D{1.1, 0.8, 1.5707963267948966},
                                 path, command));
  EXPECT_EQ(command.target_segment, 2U);
  EXPECT_NEAR(command.cross_track_error, 0.1, 1e-12);
}

TEST(Stanley, RejectsInvalidInputsWithoutChangingCommand) {
  Stanley controller;
  ASSERT_TRUE(controller.configure(StanleyConfig{1.0, 1.0, 0.1, 1.0, 0.5, 0.6}));
  StanleyCommand command{1.0, 2.0, 3.0, 4.0, 5.0, 6U};
  EXPECT_FALSE(controller.compute(StanleyPose2D{}, {{0.0, 0.0}}, command));
  EXPECT_DOUBLE_EQ(command.linear_x, 1.0);
  EXPECT_FALSE(controller.compute(StanleyPose2D{}, {
      {0.0, 0.0}, {std::numeric_limits<double>::infinity(), 0.0}}, command));
  EXPECT_DOUBLE_EQ(command.steering_angle, 2.0);
}

TEST(Stanley, ClosedLoopStraightPathConvergesFromLateralOffset) {
  Stanley controller;
  ASSERT_TRUE(controller.configure(StanleyConfig{2.0, 1.5, 0.15, 1.0, 0.5, 0.6}));
  std::vector<StanleyPathPoint2D> path;
  for (int i = 0; i <= 200; ++i) {
    path.push_back(StanleyPathPoint2D{0.1 * static_cast<double>(i), 0.0});
  }
  StanleyPose2D pose{0.0, 0.5, 0.0};
  constexpr double dt = 0.01;
  for (int step = 0; step < 1500; ++step) {
    StanleyCommand command;
    ASSERT_TRUE(controller.compute(pose, path, command));
    pose.x += command.linear_x * std::cos(pose.yaw) * dt;
    pose.y += command.linear_x * std::sin(pose.yaw) * dt;
    pose.yaw += command.angular_z * dt;
  }
  EXPECT_LT(std::abs(pose.y), 0.08);
  EXPECT_GT(pose.x, 8.0);
}

}  // namespace
