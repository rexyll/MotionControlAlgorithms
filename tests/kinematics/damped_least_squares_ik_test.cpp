#include <limits>
#include <gtest/gtest.h>
#include "mca/kinematics/damped_least_squares_ik.hpp"

namespace {
using namespace mca::kinematics;

RobotModel one_joint_model(double lower = -3.14, double upper = 3.14) {
  DhParameter p; p.name = "j"; p.type = JointType::Revolute; p.a = 1.0;
  p.limits.lower = lower; p.limits.upper = upper;
  RobotModel model;
  (void)DhModelAdapter::build("one", "base", {p}, model);
  return model;
}

TEST(DampedLeastSquaresIkTest, ConvergesToReachableTarget) {
  const auto model = one_joint_model();
  ForwardKinematicsResult target;
  ASSERT_TRUE(ForwardKinematics::solve(model, {0.7}, target));
  const auto result = DampedLeastSquaresIk::solve(model, target.pose, {0.1});
  EXPECT_EQ(result.status, IkStatus::Converged) << result.message;
  ASSERT_EQ(result.joint_positions.size(), 1U);
  EXPECT_NEAR(result.joint_positions[0], 0.7, 1e-3);
}

TEST(DampedLeastSquaresIkTest, ReducesResidualOnTwoJointChain) {
  DhParameter a; a.name = "a"; a.type = JointType::Revolute; a.a = 1.0;
  DhParameter b; b.name = "b"; b.type = JointType::Revolute; b.a = 0.7;
  RobotModel model;
  ASSERT_TRUE(DhModelAdapter::build("two", "base", {a, b}, model));
  ForwardKinematicsResult target;
  ASSERT_TRUE(ForwardKinematics::solve(model, {0.5, -0.4}, target));
  const auto result = DampedLeastSquaresIk::solve(model, target.pose, {0.0, 0.0});
  EXPECT_TRUE(result.converged()) << result.message;
  EXPECT_LT(result.position_error, 1e-5);
  EXPECT_LT(result.orientation_error, 1e-4);
}

TEST(DampedLeastSquaresIkTest, EnforcesJointLimitsForUnreachableTarget) {
  const auto model = one_joint_model(-0.2, 0.2);
  ForwardKinematicsResult target;
  ASSERT_TRUE(ForwardKinematics::solve(model, {0.8}, target));
  const auto result = DampedLeastSquaresIk::solve(model, target.pose, {0.0});
  EXPECT_EQ(result.status, IkStatus::UnreachableOrStalled);
  ASSERT_EQ(result.joint_positions.size(), 1U);
  EXPECT_GE(result.joint_positions[0], -0.2 - 1e-12);
  EXPECT_LE(result.joint_positions[0], 0.2 + 1e-12);
}

TEST(DampedLeastSquaresIkTest, RejectsInvalidDampingConfiguration) {
  const auto model = one_joint_model();
  DampedLeastSquaresIkOptions options;
  options.initial_damping = 1e-5;
  options.min_damping = 1e-3;
  const auto result = DampedLeastSquaresIk::solve(model, {}, {0.0}, options);
  EXPECT_EQ(result.status, IkStatus::InvalidInput);
}
}  // namespace
