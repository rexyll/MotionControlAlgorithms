#include <limits>
#include <gtest/gtest.h>

#include "mca/control/cbf_slew_rate_filter.hpp"
#include "mca/control/robust_cbf_slew_rate_filter.hpp"

namespace {
mca::control::DiscreteTimeCbfConfig make_config() {
  mca::control::DiscreteTimeCbfConfig config;
  config.gamma = 0.5;
  config.control_min = Eigen::Vector2d(-2.0, -2.0);
  config.control_max = Eigen::Vector2d(2.0, 2.0);
  return config;
}

mca::control::RobustCbfSlewRateInput make_input() {
  mca::control::RobustCbfSlewRateInput input;
  input.robust_prediction.prediction.barrier_values =
      Eigen::VectorXd::Constant(1, 1.0);
  input.robust_prediction.prediction.predicted_drift =
      Eigen::VectorXd::Constant(1, -1.0);
  input.robust_prediction.prediction.control_derivatives =
      Eigen::MatrixXd::Zero(1, 2);
  input.robust_prediction.prediction.control_derivatives(0, 0) = 1.0;
  input.robust_prediction.prediction.nominal_control = Eigen::Vector2d(-2.0, 0.0);
  input.robust_prediction.prediction_error_bounds = Eigen::VectorXd::Constant(1, 0.25);
  input.previous_control = Eigen::Vector2d(1.5, 0.0);
  input.max_control_delta = Eigen::Vector2d(0.5, 0.5);
  return input;
}

TEST(RobustCbfSlewRateFilter, EnforcesWorstCaseSafetyAndSlewBounds) {
  mca::control::RobustCbfSlewRateFilter filter;
  ASSERT_TRUE(filter.configure(make_config()).ok());
  const auto input = make_input();
  mca::control::DiscreteTimeCbfOutput output;
  ASSERT_TRUE(filter.filter(input, output).ok());
  EXPECT_NEAR(output.control.x(), 1.75, 1e-8);
  EXPECT_LE((output.control - input.previous_control).cwiseAbs().maxCoeff(),
            input.max_control_delta.maxCoeff() + 1e-8);
  EXPECT_GE(output.minimum_safety_margin, -1e-8);
}

TEST(RobustCbfSlewRateFilter, ZeroUncertaintyMatchesNonRobustSlewFilter) {
  mca::control::RobustCbfSlewRateFilter robust_filter;
  mca::control::CbfSlewRateFilter regular_filter;
  ASSERT_TRUE(robust_filter.configure(make_config()).ok());
  ASSERT_TRUE(regular_filter.configure(make_config()).ok());
  auto input = make_input();
  input.robust_prediction.prediction_error_bounds.setZero();

  mca::control::DiscreteTimeCbfOutput robust_output;
  mca::control::DiscreteTimeCbfOutput regular_output;
  ASSERT_TRUE(robust_filter.filter(input, robust_output).ok());
  mca::control::CbfSlewRateInput regular_input;
  regular_input.prediction = input.robust_prediction.prediction;
  regular_input.previous_control = input.previous_control;
  regular_input.max_control_delta = input.max_control_delta;
  ASSERT_TRUE(regular_filter.filter(regular_input, regular_output).ok());
  EXPECT_TRUE(robust_output.control.isApprox(regular_output.control));
  EXPECT_NEAR(robust_output.minimum_safety_margin,
              regular_output.minimum_safety_margin, 1e-10);
}

TEST(RobustCbfSlewRateFilter, UncertaintyCanMakeSlewFeasibleSetUnsafe) {
  mca::control::RobustCbfSlewRateFilter filter;
  ASSERT_TRUE(filter.configure(make_config()).ok());
  auto input = make_input();
  input.max_control_delta = Eigen::Vector2d::Constant(0.1);
  mca::control::DiscreteTimeCbfOutput output;
  output.control = Eigen::Vector2d(7.0, 8.0);
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::ConstraintViolation);
  EXPECT_DOUBLE_EQ(output.control.x(), 7.0);
  EXPECT_DOUBLE_EQ(output.control.y(), 8.0);
}

TEST(RobustCbfSlewRateFilter, RejectsInvalidUncertaintyAndRateInputsTransactionally) {
  mca::control::RobustCbfSlewRateFilter filter;
  ASSERT_TRUE(filter.configure(make_config()).ok());
  auto input = make_input();
  mca::control::DiscreteTimeCbfOutput output;
  output.control = Eigen::Vector2d(7.0, 8.0);

  input.robust_prediction.prediction_error_bounds[0] =
      std::numeric_limits<double>::infinity();
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_DOUBLE_EQ(output.control.x(), 7.0);

  input = make_input();
  input.robust_prediction.prediction_error_bounds[0] = -0.1;
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::InvalidArgument);

  input = make_input();
  input.max_control_delta[0] = -0.1;
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::InvalidArgument);

  input = make_input();
  input.robust_prediction.prediction_error_bounds.resize(2);
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_DOUBLE_EQ(output.control.x(), 7.0);
  EXPECT_DOUBLE_EQ(output.control.y(), 8.0);
}
}  // namespace
