#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "mca/path_tracking/lqr_path_tracker.hpp"

namespace {
using mca::path_tracking::LqrCommand;
using mca::path_tracking::LqrConfig;
using mca::path_tracking::LqrPathPoint2D;
using mca::path_tracking::LqrPathTracker;
using mca::path_tracking::LqrPose2D;

TEST(LqrPathTracker, RejectsInvalidConfiguration) {
  LqrPathTracker tracker;
  EXPECT_FALSE(tracker.configure(LqrConfig{0.0, 1.0, 1.0, 0.05, 1.0, 0.5, 0.6}));
  EXPECT_FALSE(tracker.configure(LqrConfig{2.0, 1.0, 1.0, 0.0, 1.0, 0.5, 0.6}));
  EXPECT_FALSE(tracker.configure(LqrConfig{2.0, 1.0, 1.0, 0.05, 0.0, 0.5, 0.6}));
  EXPECT_FALSE(tracker.configure(LqrConfig{
      2.0, 1.0, 1.0, 0.05, 1.0, 0.5,
      std::numeric_limits<double>::quiet_NaN()}));
}

TEST(LqrPathTracker, StraightPathAtCenterProducesNoCorrection) {
  LqrPathTracker tracker;
  ASSERT_TRUE(tracker.configure(LqrConfig{2.0, 1.0, 1.0, 0.05, 1.0, 0.5, 0.6}));
  LqrCommand command;
  ASSERT_TRUE(tracker.compute(LqrPose2D{1.0, 0.0, 0.0},
                              {{0.0, 0.0}, {5.0, 0.0}}, command));
  EXPECT_NEAR(command.lateral_error, 0.0, 1e-12);
  EXPECT_NEAR(command.heading_error, 0.0, 1e-12);
  EXPECT_NEAR(command.steering_angle, 0.0, 1e-12);
  EXPECT_NEAR(command.angular_z, 0.0, 1e-12);
}

TEST(LqrPathTracker, LateralAndHeadingErrorsCorrectTowardPath) {
  LqrPathTracker tracker;
  ASSERT_TRUE(tracker.configure(LqrConfig{2.0, 1.0, 1.0, 0.05, 1.0, 0.5, 0.6}));
  LqrCommand command;
  ASSERT_TRUE(tracker.compute(LqrPose2D{1.0, 0.5, 0.0},
                              {{0.0, 0.0}, {10.0, 0.0}}, command));
  EXPECT_GT(command.lateral_error, 0.0);
  EXPECT_LT(command.steering_angle, 0.0);
  EXPECT_LT(command.angular_z, 0.0);

  ASSERT_TRUE(tracker.compute(LqrPose2D{1.0, 0.0, 0.4},
                              {{0.0, 0.0}, {10.0, 0.0}}, command));
  EXPECT_NEAR(command.heading_error, 0.4, 1e-12);
  EXPECT_LT(command.steering_angle, 0.0);
}

TEST(LqrPathTracker, SelectsNearestSegmentAndSkipsDuplicateVertices) {
  LqrPathTracker tracker;
  ASSERT_TRUE(tracker.configure(LqrConfig{}));
  LqrCommand command;
  ASSERT_TRUE(tracker.compute(LqrPose2D{1.1, 0.8, 1.5707963267948966},
                              {{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {1.0, 2.0}},
                              command));
  EXPECT_EQ(command.target_segment, 2U);
  EXPECT_NEAR(command.lateral_error, -0.1, 1e-12);
}

TEST(LqrPathTracker, InvalidInputDoesNotModifyCommand) {
  LqrPathTracker tracker;
  ASSERT_TRUE(tracker.configure(LqrConfig{}));
  LqrCommand command{1.0, 2.0, 3.0, 4.0, 5.0, 6U};
  EXPECT_FALSE(tracker.compute(LqrPose2D{}, {{0.0, 0.0}}, command));
  EXPECT_DOUBLE_EQ(command.linear_x, 1.0);
  EXPECT_FALSE(tracker.compute(LqrPose2D{}, {
      {0.0, 0.0}, {std::numeric_limits<double>::infinity(), 0.0}}, command));
  EXPECT_DOUBLE_EQ(command.steering_angle, 2.0);
}

TEST(LqrPathTracker, ClosedLoopStraightPathConvergesFromLateralOffset) {
  LqrConfig config{2.0, 1.0, 0.5, 0.02, 1.0, 0.5, 0.6};
  LqrPathTracker tracker;
  ASSERT_TRUE(tracker.configure(config));
  std::vector<LqrPathPoint2D> path;
  for (int i = 0; i <= 300; ++i) {
    path.push_back(LqrPathPoint2D{0.1 * static_cast<double>(i), 0.0});
  }
  LqrPose2D pose{0.0, 0.5, 0.0};
  for (int step = 0; step < 1500; ++step) {
    LqrCommand command;
    ASSERT_TRUE(tracker.compute(pose, path, command));
    pose.x += command.linear_x * std::cos(pose.yaw) * config.sample_time;
    pose.y += command.linear_x * std::sin(pose.yaw) * config.sample_time;
    pose.yaw += command.angular_z * config.sample_time;
  }
  EXPECT_LT(std::abs(pose.y), 0.08);
  EXPECT_GT(pose.x, 8.0);
}

TEST(LqrPathTracker, SteeringIsSaturatedAtConfiguredLimit) {
  LqrPathTracker tracker;
  ASSERT_TRUE(tracker.configure(LqrConfig{10.0, 10.0, 0.1, 0.05, 1.0, 0.5, 0.2}));
  LqrCommand command;
  ASSERT_TRUE(tracker.compute(LqrPose2D{0.0, 5.0, 1.0},
                              {{0.0, 0.0}, {10.0, 0.0}}, command));
  EXPECT_NEAR(command.steering_angle, -0.2, 1e-12);
}

}  // namespace
