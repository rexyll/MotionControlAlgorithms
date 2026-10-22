#include "mca/legged/walking_manipulation.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using namespace mca::legged;

WalkingManipulationInput base_input() {
  WalkingManipulationInput input;
  input.variables = 2;
  input.locomotion_jacobian = Eigen::Matrix2d::Identity();
  input.locomotion_target = Eigen::Vector2d(0.0, 0.0);
  input.manipulation_jacobian = Eigen::Matrix2d::Identity();
  input.manipulation_target = Eigen::Vector2d(1.0, -0.5);
  input.locomotion_weight = 20.0;
  input.manipulation_weight = 1.0;
  input.regularization = 1e-8;
  input.iterations = 250;
  return input;
}

TEST(WalkingManipulation, PrioritizesLocomotionStabilityOverArmTask) {
  auto input = base_input();
  WalkingManipulationOutput output;
  ASSERT_TRUE(WalkingManipulationController::solve(input, output));
  EXPECT_NEAR(output.generalized_command[0], 1.0 / 21.0, 2e-3);
  EXPECT_NEAR(output.generalized_command[1], -0.5 / 21.0, 2e-3);
  EXPECT_LT(output.locomotion_residual_norm, output.manipulation_residual_norm);
}

TEST(WalkingManipulation, AppliesGeneralizedCommandBounds) {
  auto input = base_input();
  input.locomotion_weight = 1.0;
  input.manipulation_weight = 10.0;
  input.lower_bounds = Eigen::Vector2d(-0.2, -0.2);
  input.upper_bounds = Eigen::Vector2d(0.3, 0.3);
  WalkingManipulationOutput output;
  ASSERT_TRUE(WalkingManipulationController::solve(input, output));
  EXPECT_NEAR(output.generalized_command[0], 0.3, 1e-8);
  EXPECT_NEAR(output.generalized_command[1], -0.2, 1e-8);
}

TEST(WalkingManipulation, RejectsNonFiniteTargetTransactionally) {
  auto input = base_input();
  input.manipulation_target[0] = std::numeric_limits<double>::quiet_NaN();
  WalkingManipulationOutput output;
  output.generalized_command = Eigen::Vector2d(7.0, 8.0);
  output.objective = 42.0;
  std::string error;
  EXPECT_FALSE(WalkingManipulationController::solve(input, output, &error));
  EXPECT_FALSE(error.empty());
  EXPECT_DOUBLE_EQ(output.generalized_command[0], 7.0);
  EXPECT_DOUBLE_EQ(output.generalized_command[1], 8.0);
  EXPECT_DOUBLE_EQ(output.objective, 42.0);
}

TEST(WalkingManipulation, RejectsMismatchedDimensions) {
  auto input = base_input();
  input.manipulation_jacobian.resize(3, 2);
  WalkingManipulationOutput output;
  EXPECT_FALSE(WalkingManipulationController::solve(input, output));
}
}  // namespace
