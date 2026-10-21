#include "mca/legged/whole_body_control.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using mca::legged::WholeBodyControl;
using mca::legged::WholeBodyControlInput;
using mca::legged::WholeBodyTask;

TEST(WholeBodyControl, AggregatesFeedforwardTaskAndSecondaryTorques) {
  WholeBodyControlInput input;
  input.dof = 2;
  input.feedforward_torque = Eigen::Vector2d(1.0, 2.0);
  input.secondary_torque = Eigen::Vector2d(-0.5, 0.5);
  WholeBodyTask task;
  task.jacobian.resize(1, 2);
  task.jacobian << 2.0, -3.0;
  task.task_wrench = Eigen::VectorXd::Constant(1, 4.0);
  input.tasks.push_back(task);
  Eigen::VectorXd output;
  ASSERT_TRUE(WholeBodyControl::compute(input, output));
  EXPECT_DOUBLE_EQ(output[0], 8.5);
  EXPECT_DOUBLE_EQ(output[1], -9.5);
}

TEST(WholeBodyControl, AppliesPerJointTorqueLimits) {
  WholeBodyControlInput input;
  input.dof = 2;
  input.feedforward_torque = Eigen::Vector2d(8.0, -9.0);
  input.secondary_torque = Eigen::Vector2d::Zero();
  input.torque_limits = Eigen::Vector2d(3.0, 4.0);
  Eigen::VectorXd output;
  ASSERT_TRUE(WholeBodyControl::compute(input, output));
  EXPECT_DOUBLE_EQ(output[0], 3.0);
  EXPECT_DOUBLE_EQ(output[1], -4.0);
}

TEST(WholeBodyControl, RejectsBadTaskTransactionally) {
  WholeBodyControlInput input;
  input.dof = 2;
  input.feedforward_torque = Eigen::Vector2d(1.0, 2.0);
  input.secondary_torque = Eigen::Vector2d::Zero();
  WholeBodyTask task;
  task.jacobian = Eigen::MatrixXd::Identity(2, 3);
  task.task_wrench = Eigen::Vector2d::Ones();
  input.tasks.push_back(task);
  Eigen::VectorXd output = Eigen::Vector2d(7.0, 8.0);
  std::string error;
  EXPECT_FALSE(WholeBodyControl::compute(input, output, &error));
  EXPECT_EQ(error, "whole-body task dimensions are inconsistent");
  ASSERT_EQ(output.size(), 2);
  EXPECT_DOUBLE_EQ(output[0], 7.0);
  EXPECT_DOUBLE_EQ(output[1], 8.0);
}

TEST(WholeBodyControl, RejectsNonFiniteAndInvalidLimits) {
  WholeBodyControlInput input;
  input.dof = 1;
  input.feedforward_torque = Eigen::VectorXd::Constant(
      1, std::numeric_limits<double>::quiet_NaN());
  input.secondary_torque = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd output = Eigen::VectorXd::Constant(1, 5.0);
  EXPECT_FALSE(WholeBodyControl::compute(input, output));
  EXPECT_DOUBLE_EQ(output[0], 5.0);
  input.feedforward_torque.setZero();
  input.torque_limits = Eigen::VectorXd::Constant(1, -1.0);
  EXPECT_FALSE(WholeBodyControl::compute(input, output));
  EXPECT_DOUBLE_EQ(output[0], 5.0);
}
}  // namespace
