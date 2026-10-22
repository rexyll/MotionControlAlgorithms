#include <limits>
#include <gtest/gtest.h>

#include "mca/control/robust_discrete_time_cbf_filter.hpp"

namespace {
mca::control::RobustDiscreteTimeCbfFilter make_filter() {
  mca::control::RobustDiscreteTimeCbfFilter filter;
  mca::control::DiscreteTimeCbfConfig config;
  config.gamma = 0.5;
  config.control_min = Eigen::Vector2d(-2.0, -2.0);
  config.control_max = Eigen::Vector2d(2.0, 2.0);
  EXPECT_TRUE(filter.configure(config).ok());
  return filter;
}

mca::control::RobustDiscreteTimeCbfInput make_input(double uncertainty) {
  mca::control::RobustDiscreteTimeCbfInput input;
  input.prediction.barrier_values = Eigen::VectorXd::Constant(1, 1.0);
  input.prediction.predicted_drift = Eigen::VectorXd::Constant(1, -1.0);
  input.prediction.control_derivatives = Eigen::MatrixXd::Zero(1, 2);
  input.prediction.control_derivatives(0, 0) = 1.0;
  input.prediction.nominal_control = Eigen::Vector2d(-2.0, 0.0);
  input.prediction_error_bounds = Eigen::VectorXd::Constant(1, uncertainty);
  return input;
}

TEST(RobustDiscreteTimeCbfFilter, TightensControlForBoundedPredictionError) {
  auto filter = make_filter();
  auto input = make_input(0.2);
  mca::control::DiscreteTimeCbfOutput output;
  ASSERT_TRUE(filter.filter(input, output).ok());
  EXPECT_NEAR(output.control.x(), 1.7, 1e-8);
  EXPECT_GE(output.minimum_safety_margin, -1e-8);
  EXPECT_TRUE(output.modified);
}

TEST(RobustDiscreteTimeCbfFilter, ZeroUncertaintyMatchesNominalDiscreteFilter) {
  auto robust = make_filter();
  auto input = make_input(0.0);
  mca::control::DiscreteTimeCbfOutput robust_output;
  ASSERT_TRUE(robust.filter(input, robust_output).ok());

  mca::control::DiscreteTimeCbfFilter nominal;
  mca::control::DiscreteTimeCbfConfig config;
  config.gamma = 0.5;
  config.control_min = Eigen::Vector2d(-2.0, -2.0);
  config.control_max = Eigen::Vector2d(2.0, 2.0);
  ASSERT_TRUE(nominal.configure(config).ok());
  mca::control::DiscreteTimeCbfOutput nominal_output;
  ASSERT_TRUE(nominal.filter(input.prediction, nominal_output).ok());
  EXPECT_TRUE(robust_output.control.isApprox(nominal_output.control));
  EXPECT_NEAR(robust_output.minimum_safety_margin,
              nominal_output.minimum_safety_margin, 1e-10);
}

TEST(RobustDiscreteTimeCbfFilter, RejectsNegativeAndNonFiniteErrorBounds) {
  auto filter = make_filter();
  auto input = make_input(-0.1);
  mca::control::DiscreteTimeCbfOutput output;
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::InvalidArgument);
  input.prediction_error_bounds[0] =
      std::numeric_limits<double>::infinity();
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::InvalidArgument);
}

TEST(RobustDiscreteTimeCbfFilter, RejectsPredictionDimensionMismatch) {
  auto filter = make_filter();
  auto input = make_input(0.1);
  input.prediction.predicted_drift.resize(2);
  mca::control::DiscreteTimeCbfOutput output;
  output.control = Eigen::Vector2d(7.0, 8.0);
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_DOUBLE_EQ(output.control.x(), 7.0);
  EXPECT_DOUBLE_EQ(output.control.y(), 8.0);
}

TEST(RobustDiscreteTimeCbfFilter, InfeasibleRobustConstraintPreservesOutput) {
  auto filter = make_filter();
  auto input = make_input(100.0);
  mca::control::DiscreteTimeCbfOutput output;
  output.control = Eigen::Vector2d(7.0, 8.0);
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::ConstraintViolation);
  EXPECT_DOUBLE_EQ(output.control.x(), 7.0);
  EXPECT_DOUBLE_EQ(output.control.y(), 8.0);
}
}  // namespace
