#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "mca/legged/locomotion_mpc.hpp"

namespace {

mca::legged::LocomotionMpcInput base_input(int horizon = 20) {
  mca::legged::LocomotionMpcInput input;
  input.initial_state.setZero();
  input.desired_states = Eigen::MatrixXd::Zero(4, horizon + 1);
  input.dt = 0.1;
  input.state_weight << 4.0, 4.0, 1.0, 1.0;
  input.terminal_weight << 8.0, 8.0, 2.0, 2.0;
  input.control_weight << 0.1, 0.1;
  input.acceleration_limits << 2.0, 2.0;
  input.iterations = 120;
  input.step_size = 0.01;
  return input;
}

}  // namespace

TEST(LocomotionMpcTest, StandingProducesNearZeroAcceleration) {
  auto input = base_input();
  mca::legged::LocomotionMpcOutput output;
  ASSERT_TRUE(mca::legged::LocomotionMpc().solve(input, &output));
  EXPECT_NEAR(output.acceleration.x(), 0.0, 1e-6);
  EXPECT_NEAR(output.acceleration.y(), 0.0, 1e-6);
  EXPECT_LT(output.objective, 1e-8);
}

TEST(LocomotionMpcTest, WalkingForwardProducesPositiveXAcceleration) {
  auto input = base_input();
  for (int k = 0; k < input.desired_states.cols(); ++k) {
    const double t = k * input.dt;
    input.desired_states(0, k) = 0.5 * t * t;
    input.desired_states(2, k) = t;
  }

  mca::legged::LocomotionMpcOutput output;
  ASSERT_TRUE(mca::legged::LocomotionMpc().solve(input, &output));
  EXPECT_GT(output.acceleration.x(), 0.0);
  EXPECT_NEAR(output.acceleration.y(), 0.0, 1e-8);
  EXPECT_LE(std::abs(output.acceleration.x()), input.acceleration_limits.x() + 1e-12);
}

TEST(LocomotionMpcTest, DisturbanceRecoveryRequestsNegativeAcceleration) {
  auto input = base_input();
  input.initial_state(2) = 1.0;
  input.desired_states.setZero();

  mca::legged::LocomotionMpcOutput output;
  ASSERT_TRUE(mca::legged::LocomotionMpc().solve(input, &output));
  EXPECT_LT(output.acceleration.x(), 0.0);
  EXPECT_NEAR(output.acceleration.y(), 0.0, 1e-8);
}

TEST(LocomotionMpcTest, InvalidInputPreservesOutput) {
  auto input = base_input();
  input.dt = -0.1;
  mca::legged::LocomotionMpcOutput output;
  output.acceleration << 3.0, -4.0;
  output.objective = 7.0;
  output.iterations = 9;
  ASSERT_FALSE(mca::legged::LocomotionMpc().solve(input, &output));
  EXPECT_EQ(output.acceleration.x(), 3.0);
  EXPECT_EQ(output.acceleration.y(), -4.0);
  EXPECT_EQ(output.objective, 7.0);
  EXPECT_EQ(output.iterations, 9);
}

TEST(LocomotionMpcTest, NonFiniteReferenceRejected) {
  auto input = base_input();
  input.desired_states(0, 3) = std::numeric_limits<double>::quiet_NaN();
  mca::legged::LocomotionMpcOutput output;
  output.acceleration << 1.0, 2.0;
  ASSERT_FALSE(mca::legged::LocomotionMpc().solve(input, &output));
  EXPECT_EQ(output.acceleration.x(), 1.0);
  EXPECT_EQ(output.acceleration.y(), 2.0);
}
