#include "mca/multi_robot/leader_follower.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using mca::multi_robot::LeaderFollowerInput;
using mca::multi_robot::LeaderFollowerOutput;
using mca::multi_robot::leader_follower_control;

LeaderFollowerInput line_input() {
  LeaderFollowerInput input;
  input.positions.resize(3, 2);
  input.positions << 0.0, 0.0,
                     2.0, 0.0,
                    -1.0, 0.0;
  input.desired_offsets = Eigen::MatrixXd::Zero(3, 2);
  input.desired_offsets.row(1) << 1.0, 0.0;
  input.desired_offsets.row(2) << -1.0, 0.0;
  input.leader_index = 0;
  input.leader_velocity.resize(2);
  input.leader_velocity << 0.2, -0.1;
  input.gain = 1.0;
  input.max_follower_speed = 10.0;
  return input;
}

TEST(LeaderFollower, LeaderCommandIsPassedThroughAndFollowersTrackOffsets) {
  const auto input = line_input();
  LeaderFollowerOutput output;
  ASSERT_TRUE(leader_follower_control(input, output));
  EXPECT_DOUBLE_EQ(output.velocities(0, 0), 0.2);
  EXPECT_DOUBLE_EQ(output.velocities(0, 1), -0.1);
  EXPECT_NEAR(output.velocities(1, 0), -0.8, 1e-12);
  EXPECT_NEAR(output.velocities(2, 0), 0.2, 1e-12);
}

TEST(LeaderFollower, SupportsLeaderAtNonzeroIndex) {
  LeaderFollowerInput input = line_input();
  input.leader_index = 1;
  input.positions.row(1) << 4.0, 3.0;
  input.desired_offsets.row(0) << -2.0, 0.0;
  input.desired_offsets.row(2) << 2.0, 0.0;
  LeaderFollowerOutput output;
  ASSERT_TRUE(leader_follower_control(input, output));
  EXPECT_DOUBLE_EQ(output.velocities(1, 0), input.leader_velocity(0));
  EXPECT_DOUBLE_EQ(output.velocities(1, 1), input.leader_velocity(1));
  EXPECT_TRUE(output.velocities.allFinite());
}

TEST(LeaderFollower, SaturatesEachFollowerCommand) {
  LeaderFollowerInput input = line_input();
  input.gain = 100.0;
  input.max_follower_speed = 0.3;
  LeaderFollowerOutput output;
  ASSERT_TRUE(leader_follower_control(input, output));
  for (Eigen::Index i = 0; i < output.velocities.rows(); ++i) {
    if (i != input.leader_index)
      EXPECT_LE(output.velocities.row(i).norm(), 0.3 + 1e-12);
  }
}

TEST(LeaderFollower, RejectsInvalidInputsTransactionally) {
  LeaderFollowerInput input = line_input();
  LeaderFollowerOutput output;
  output.velocities = Eigen::MatrixXd::Constant(1, 1, 17.0);
  input.leader_index = 9;
  std::string error;
  EXPECT_FALSE(leader_follower_control(input, output, &error));
  EXPECT_FALSE(error.empty());
  ASSERT_EQ(output.velocities.rows(), 1);
  EXPECT_DOUBLE_EQ(output.velocities(0, 0), 17.0);
  input = line_input();
  input.positions(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(leader_follower_control(input, output));
  input = line_input();
  input.leader_velocity.resize(3);
  EXPECT_FALSE(leader_follower_control(input, output));
}
}  // namespace
