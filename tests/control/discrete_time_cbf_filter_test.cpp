#include <limits>
#include <gtest/gtest.h>
#include "mca/control/discrete_time_cbf_filter.hpp"

namespace {
mca::control::DiscreteTimeCbfFilter make_filter(double gamma = 0.2) {
  mca::control::DiscreteTimeCbfFilter filter;
  mca::control::DiscreteTimeCbfConfig config;
  config.gamma = gamma;
  config.control_min = Eigen::Vector2d(-2.0, -2.0);
  config.control_max = Eigen::Vector2d(2.0, 2.0);
  EXPECT_TRUE(filter.configure(config).ok());
  return filter;
}

TEST(DiscreteTimeCbfFilter, EnforcesOneStepBarrierCondition) {
  auto filter = make_filter(0.5);
  mca::control::DiscreteTimeCbfInput input;
  input.barrier_values = Eigen::VectorXd::Constant(1, 1.0);
  input.predicted_drift = Eigen::VectorXd::Constant(1, -1.0);
  input.control_derivatives = Eigen::MatrixXd::Zero(1, 2);
  input.control_derivatives(0, 0) = 1.0;
  input.nominal_control = Eigen::Vector2d(-2.0, 0.0);
  mca::control::DiscreteTimeCbfOutput output;
  ASSERT_TRUE(filter.filter(input, output).ok());
  EXPECT_NEAR(output.control.x(), 1.5, 1e-8);
  EXPECT_GE(output.minimum_safety_margin, -1e-8);
  EXPECT_TRUE(output.modified);
}

TEST(DiscreteTimeCbfFilter, LeavesFeasibleNominalUnchanged) {
  auto filter = make_filter();
  mca::control::DiscreteTimeCbfInput input;
  input.barrier_values = Eigen::VectorXd::Constant(1, 1.0);
  input.predicted_drift = Eigen::VectorXd::Constant(1, 1.0);
  input.control_derivatives = Eigen::MatrixXd::Ones(1, 2);
  input.nominal_control = Eigen::Vector2d(0.2, 0.1);
  mca::control::DiscreteTimeCbfOutput output;
  ASSERT_TRUE(filter.filter(input, output).ok());
  EXPECT_TRUE(output.control.isApprox(input.nominal_control));
  EXPECT_FALSE(output.modified);
}

TEST(DiscreteTimeCbfFilter, AppliesSafetyMarginAndActuatorBounds) {
  mca::control::DiscreteTimeCbfFilter filter;
  mca::control::DiscreteTimeCbfConfig config;
  config.gamma = 1.0;
  config.safety_margin = 0.25;
  config.control_min = Eigen::Vector2d(-1.0, -1.0);
  config.control_max = Eigen::Vector2d(1.0, 1.0);
  ASSERT_TRUE(filter.configure(config).ok());
  mca::control::DiscreteTimeCbfInput input;
  input.barrier_values = Eigen::VectorXd::Zero(1);
  input.predicted_drift = Eigen::VectorXd::Constant(1, -0.5);
  input.control_derivatives = Eigen::MatrixXd::Zero(1, 2);
  input.control_derivatives(0, 0) = 1.0;
  input.nominal_control = Eigen::Vector2d(-1.0, 0.0);
  mca::control::DiscreteTimeCbfOutput output;
  ASSERT_TRUE(filter.filter(input, output).ok());
  EXPECT_NEAR(output.control.x(), 0.75, 1e-8);
  EXPECT_GE(output.minimum_safety_margin, 0.25 - 1e-8);
}

TEST(DiscreteTimeCbfFilter, InfeasibleConstraintsPreserveOutput) {
  auto filter = make_filter();
  mca::control::DiscreteTimeCbfInput input;
  input.barrier_values = Eigen::VectorXd::Zero(2);
  input.predicted_drift = Eigen::VectorXd::Constant(2, -10.0);
  input.control_derivatives = Eigen::MatrixXd::Zero(2, 2);
  input.nominal_control = Eigen::Vector2d::Zero();
  mca::control::DiscreteTimeCbfOutput output;
  output.control = Eigen::Vector2d(7.0, 8.0);
  EXPECT_EQ(filter.filter(input, output).code, mca::core::StatusCode::ConstraintViolation);
  EXPECT_DOUBLE_EQ(output.control.x(), 7.0);
  EXPECT_DOUBLE_EQ(output.control.y(), 8.0);
}

TEST(DiscreteTimeCbfFilter, RejectsInvalidConfigurationAndInput) {
  mca::control::DiscreteTimeCbfFilter filter;
  mca::control::DiscreteTimeCbfConfig bad;
  bad.gamma = 0.0;
  bad.control_min = Eigen::VectorXd::Zero(1);
  bad.control_max = Eigen::VectorXd::Ones(1);
  EXPECT_EQ(filter.configure(bad).code, mca::core::StatusCode::InvalidArgument);

  auto valid = make_filter();
  mca::control::DiscreteTimeCbfInput input;
  input.barrier_values = Eigen::VectorXd::Ones(1);
  input.predicted_drift = Eigen::VectorXd::Ones(1);
  input.control_derivatives = Eigen::MatrixXd::Constant(
      1, 2, std::numeric_limits<double>::quiet_NaN());
  input.nominal_control = Eigen::Vector2d::Zero();
  mca::control::DiscreteTimeCbfOutput output;
  EXPECT_EQ(valid.filter(input, output).code, mca::core::StatusCode::InvalidArgument);
}
}  // namespace
