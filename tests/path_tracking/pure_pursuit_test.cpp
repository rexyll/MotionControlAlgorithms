#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "mca/path_tracking/pure_pursuit.hpp"

namespace {

using mca::path_tracking::PathPoint2D;
using mca::path_tracking::Pose2D;
using mca::path_tracking::PurePursuit;
using mca::path_tracking::PurePursuitCommand;
using mca::path_tracking::PurePursuitConfig;

TEST(PurePursuit, RejectsInvalidConfiguration) {
  PurePursuit controller;
  EXPECT_FALSE(controller.configure(PurePursuitConfig{0.0, 1.0}));
  EXPECT_FALSE(controller.configure(PurePursuitConfig{-1.0, 1.0}));
  EXPECT_FALSE(controller.configure(
      PurePursuitConfig{std::numeric_limits<double>::quiet_NaN(), 1.0}));
}

TEST(PurePursuit, StraightPathProducesZeroCurvature) {
  PurePursuit controller;
  ASSERT_TRUE(controller.configure(PurePursuitConfig{2.0, 1.5}));

  const std::vector<PathPoint2D> path{{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}};
  PurePursuitCommand command;
  ASSERT_TRUE(controller.compute(Pose2D{0.0, 0.0, 0.0}, path, command));

  EXPECT_NEAR(command.linear_x, 1.5, 1e-12);
  EXPECT_NEAR(command.curvature, 0.0, 1e-12);
  EXPECT_NEAR(command.angular_z, 0.0, 1e-12);
  EXPECT_EQ(command.target_index, 1U);
}

TEST(PurePursuit, PositiveLateralTargetProducesPositiveTurn) {
  PurePursuit controller;
  ASSERT_TRUE(controller.configure(PurePursuitConfig{2.0, 1.0}));

  const std::vector<PathPoint2D> path{{0.0, 0.0}, {2.0, 1.0}, {4.0, 1.0}};
  PurePursuitCommand command;
  ASSERT_TRUE(controller.compute(Pose2D{0.0, 0.0, 0.0}, path, command));

  EXPECT_GT(command.curvature, 0.0);
  EXPECT_GT(command.angular_z, 0.0);
}

TEST(PurePursuit, VehicleHeadingRotatesTargetIntoVehicleFrame) {
  PurePursuit controller;
  ASSERT_TRUE(controller.configure(PurePursuitConfig{1.0, 2.0}));

  const std::vector<PathPoint2D> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  PurePursuitCommand command;
  ASSERT_TRUE(controller.compute(
      Pose2D{0.0, 0.0, 0.5 * std::acos(-1.0)}, path, command));

  EXPECT_NEAR(command.curvature, -2.0, 1e-12);
  EXPECT_NEAR(command.angular_z, -4.0, 1e-12);
}

TEST(PurePursuit, LookaheadInterpolatesWithinPathSegment) {
  PurePursuit controller;
  ASSERT_TRUE(controller.configure(PurePursuitConfig{1.0, 1.0}));

  const std::vector<PathPoint2D> path{{0.0, 0.0}, {4.0, 0.0}};
  PurePursuitCommand command;
  ASSERT_TRUE(controller.compute(Pose2D{0.0, 0.0, 0.0}, path, command));

  EXPECT_EQ(command.target_index, 1U);
  EXPECT_NEAR(command.curvature, 0.0, 1e-12);
}

TEST(PurePursuit, UsesFinalPointWhenPathEndsBeforeLookahead) {
  PurePursuit controller;
  ASSERT_TRUE(controller.configure(PurePursuitConfig{10.0, 1.0}));

  const std::vector<PathPoint2D> path{{0.0, 0.0}, {1.0, 0.0}};
  PurePursuitCommand command;
  ASSERT_TRUE(controller.compute(Pose2D{0.0, 0.0, 0.0}, path, command));

  EXPECT_EQ(command.target_index, 1U);
  EXPECT_NEAR(command.curvature, 0.0, 1e-12);
}

TEST(PurePursuit, RejectsInvalidInputs) {
  PurePursuit controller;
  ASSERT_TRUE(controller.configure(PurePursuitConfig{1.0, 1.0}));

  PurePursuitCommand command{1.0, 2.0, 3.0, 4U};
  const std::vector<PathPoint2D> empty_path;
  EXPECT_FALSE(controller.compute(Pose2D{}, empty_path, command));
  EXPECT_DOUBLE_EQ(command.linear_x, 1.0);

  const std::vector<PathPoint2D> invalid_path{
      {0.0, 0.0}, {std::numeric_limits<double>::infinity(), 0.0}};
  EXPECT_FALSE(controller.compute(Pose2D{}, invalid_path, command));
}

TEST(PurePursuit, ClosedLoopStraightPathConvergesFromLateralOffset) {
  PurePursuit controller;
  ASSERT_TRUE(controller.configure(PurePursuitConfig{1.0, 1.0}));

  std::vector<PathPoint2D> path;
  for (int i = 0; i <= 100; ++i) {
    path.push_back(PathPoint2D{0.1 * static_cast<double>(i), 0.0});
  }

  Pose2D pose{0.0, 0.5, 0.0};
  constexpr double dt = 0.01;
  for (int step = 0; step < 1200; ++step) {
    PurePursuitCommand command;
    ASSERT_TRUE(controller.compute(pose, path, command));
    pose.x += command.linear_x * std::cos(pose.yaw) * dt;
    pose.y += command.linear_x * std::sin(pose.yaw) * dt;
    pose.yaw += command.angular_z * dt;
  }

  EXPECT_LT(std::abs(pose.y), 0.06);
  EXPECT_GT(pose.x, 8.0);
}

}  // namespace
