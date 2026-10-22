#include <limits>
#include <gtest/gtest.h>

#include "mca/control/cbf_slew_rate_filter.hpp"

namespace {
mca::control::DiscreteTimeCbfConfig make_config() {
  mca::control::DiscreteTimeCbfConfig config;
  config.gamma = 0.5;
  config.control_min = Eigen::Vector2d(-2.0, -2.0);
  config.control_max = Eigen::Vector2d(2.0, 2.0);
  return config;
}

mca::control::CbfSlewRateInput make_input() {
  mca::control::CbfSlewRateInput input;
  input.prediction.barrier_values = Eigen::VectorXd::Constant(1, 1.0);
  input.prediction.predicted_drift = Eigen::VectorXd::Constant(1, -1.0);
  input.prediction.control_derivatives = Eigen::MatrixXd::Zero(1, 2);
  input.prediction.control_derivatives(0, 0) = 1.0;
  input.prediction.nominal_control = Eigen::Vector2d(-2.0, 0.0);
  input.previous_control = Eigen::Vector2d(1.5, 0.0);
  input.max_control_delta = Eigen::Vector2d(0.5, 0.5);
  return input;
}

TEST(CbfSlewRateFilter, EnforcesSlewBoundsAndSafetyProjection) {
  mca::control::CbfSlewRateFilter filter;
  ASSERT_TRUE(filter.configure(make_config()).ok());
  auto input = make_input();
  mca::control::DiscreteTimeCbfOutput output;
  ASSERT_TRUE(filter.filter(input, output).ok());
  EXPECT_NEAR(output.control.x(), 1.5, 1e-8);
  EXPECT_LE((output.control - input.previous_control).cwiseAbs().maxCoeff(),
            input.max_control_delta.maxCoeff() + 1e-8);
  EXPECT_GE(output.minimum_safety_margin, -1e-8);
}

TEST(CbfSlewRateFilter, MatchesNominalFilterWhenSlewBoundsAreInactive) {
  mca::control::CbfSlewRateFilter slew_filter;
  ASSERT_TRUE(slew_filter.configure(make_config()).ok());
  auto input = make_input();
  input.previous_control = Eigen::Vector2d::Zero();
  input.max_control_delta = Eigen::Vector2d::Constant(10.0);
  mca::control::DiscreteTimeCbfOutput slew_output;
  ASSERT_TRUE(slew_filter.filter(input, slew_output).ok());

  mca::control::DiscreteTimeCbfFilter nominal_filter;
  ASSERT_TRUE(nominal_filter.configure(make_config()).ok());
  mca::control::DiscreteTimeCbfOutput nominal_output;
  ASSERT_TRUE(nominal_filter.filter(input.prediction, nominal_output).ok());
  EXPECT_TRUE(slew_output.control.isApprox(nominal_output.control));
  EXPECT_NEAR(slew_output.minimum_safety_margin,
              nominal_output.minimum_safety_margin, 1e-10);
}

TEST(CbfSlewRateFilter, RejectsInfeasibleSlewIntersectionTransactionally) {
  mca::control::CbfSlewRateFilter filter;
  ASSERT_TRUE(filter.configure(make_config()).ok());
  auto input = make_input();
  input.previous_control = Eigen::Vector2d(10.0, 0.0);
  input.max_control_delta = Eigen::Vector2d(0.1, 0.1);
  mca::control::DiscreteTimeCbfOutput output;
  output.control = Eigen::Vector2d(7.0, 8.0);
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::ConstraintViolation);
  EXPECT_DOUBLE_EQ(output.control.x(), 7.0);
  EXPECT_DOUBLE_EQ(output.control.y(), 8.0);
}

TEST(CbfSlewRateFilter, RejectsInfeasibleSafetyProjectionTransactionally) {
  mca::control::CbfSlewRateFilter filter;
  ASSERT_TRUE(filter.configure(make_config()).ok());
  auto input = make_input();
  input.previous_control = Eigen::Vector2d::Zero();
  input.max_control_delta = Eigen::Vector2d::Constant(0.1);
  mca::control::DiscreteTimeCbfOutput output;
  output.control = Eigen::Vector2d(7.0, 8.0);
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::ConstraintViolation);
  EXPECT_DOUBLE_EQ(output.control.x(), 7.0);
  EXPECT_DOUBLE_EQ(output.control.y(), 8.0);
}

TEST(CbfSlewRateFilter, RejectsInvalidRateBoundsTransactionally) {
  mca::control::CbfSlewRateFilter filter;
  ASSERT_TRUE(filter.configure(make_config()).ok());
  auto input = make_input();
  input.max_control_delta[0] = std::numeric_limits<double>::infinity();
  mca::control::DiscreteTimeCbfOutput output;
  output.control = Eigen::Vector2d(7.0, 8.0);
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_DOUBLE_EQ(output.control.x(), 7.0);
  EXPECT_DOUBLE_EQ(output.control.y(), 8.0);

  input = make_input();
  input.previous_control.resize(1);
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::InvalidArgument);
}
}  // namespace
