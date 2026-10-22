#include <gtest/gtest.h>

#include "mca/mca.hpp"

namespace {
mca::control::DiscreteTimeCbfConfig make_config() {
  mca::control::DiscreteTimeCbfConfig config;
  config.gamma = 0.5;
  config.control_min = Eigen::Vector2d(-2.0, -2.0);
  config.control_max = Eigen::Vector2d(2.0, 2.0);
  return config;
}

mca::control::DiscreteTimeCbfInput make_prediction() {
  mca::control::DiscreteTimeCbfInput input;
  input.barrier_values = Eigen::VectorXd::Constant(1, 1.0);
  input.predicted_drift = Eigen::VectorXd::Constant(1, -1.0);
  input.control_derivatives = Eigen::MatrixXd::Zero(1, 2);
  input.control_derivatives(0, 0) = 1.0;
  input.nominal_control = Eigen::Vector2d(-2.0, 0.0);
  return input;
}

TEST(CbfSafetyFilterIntegration, PublicHeaderExportsAllDiscreteTimeVariants) {
  const auto config = make_config();
  const auto prediction = make_prediction();

  mca::control::DiscreteTimeCbfFilter nominal;
  mca::control::RobustDiscreteTimeCbfFilter robust;
  mca::control::CbfSlewRateFilter slew;
  mca::control::RobustCbfSlewRateFilter robust_slew;
  ASSERT_TRUE(nominal.configure(config).ok());
  ASSERT_TRUE(robust.configure(config).ok());
  ASSERT_TRUE(slew.configure(config).ok());
  ASSERT_TRUE(robust_slew.configure(config).ok());

  mca::control::DiscreteTimeCbfOutput nominal_output;
  ASSERT_TRUE(nominal.filter(prediction, nominal_output).ok());

  mca::control::RobustDiscreteTimeCbfInput robust_input;
  robust_input.prediction = prediction;
  robust_input.prediction_error_bounds = Eigen::VectorXd::Zero(1);
  mca::control::DiscreteTimeCbfOutput robust_output;
  ASSERT_TRUE(robust.filter(robust_input, robust_output).ok());
  EXPECT_TRUE(robust_output.control.isApprox(nominal_output.control));

  mca::control::CbfSlewRateInput slew_input;
  slew_input.prediction = prediction;
  slew_input.previous_control = Eigen::Vector2d::Zero();
  slew_input.max_control_delta = Eigen::Vector2d::Constant(10.0);
  mca::control::DiscreteTimeCbfOutput slew_output;
  ASSERT_TRUE(slew.filter(slew_input, slew_output).ok());
  EXPECT_TRUE(slew_output.control.isApprox(nominal_output.control));

  mca::control::RobustCbfSlewRateInput robust_slew_input;
  robust_slew_input.robust_prediction = robust_input;
  robust_slew_input.previous_control = slew_input.previous_control;
  robust_slew_input.max_control_delta = slew_input.max_control_delta;
  mca::control::DiscreteTimeCbfOutput robust_slew_output;
  ASSERT_TRUE(robust_slew.filter(robust_slew_input, robust_slew_output).ok());
  EXPECT_TRUE(robust_slew_output.control.isApprox(nominal_output.control));
}

TEST(CbfSafetyFilterIntegration, AllVariantsPreserveOutputOnFailure) {
  const auto config = make_config();
  auto prediction = make_prediction();
  prediction.nominal_control = Eigen::Vector2d::Zero();

  mca::control::DiscreteTimeCbfOutput sentinel;
  sentinel.control = Eigen::Vector2d(7.0, 8.0);

  mca::control::DiscreteTimeCbfFilter nominal;
  ASSERT_TRUE(nominal.configure(config).ok());
  EXPECT_EQ(nominal.filter(prediction, sentinel).code,
            mca::core::StatusCode::ConstraintViolation);
  EXPECT_TRUE(sentinel.control.isApprox(Eigen::Vector2d(7.0, 8.0)));

  mca::control::RobustDiscreteTimeCbfFilter robust;
  ASSERT_TRUE(robust.configure(config).ok());
  mca::control::RobustDiscreteTimeCbfInput robust_input;
  robust_input.prediction = prediction;
  robust_input.prediction_error_bounds = Eigen::VectorXd::Constant(1, 10.0);
  EXPECT_EQ(robust.filter(robust_input, sentinel).code,
            mca::core::StatusCode::ConstraintViolation);
  EXPECT_TRUE(sentinel.control.isApprox(Eigen::Vector2d(7.0, 8.0)));

  mca::control::CbfSlewRateFilter slew;
  ASSERT_TRUE(slew.configure(config).ok());
  mca::control::CbfSlewRateInput slew_input;
  slew_input.prediction = prediction;
  slew_input.previous_control = Eigen::Vector2d::Zero();
  slew_input.max_control_delta = Eigen::Vector2d::Constant(0.1);
  EXPECT_EQ(slew.filter(slew_input, sentinel).code,
            mca::core::StatusCode::ConstraintViolation);
  EXPECT_TRUE(sentinel.control.isApprox(Eigen::Vector2d(7.0, 8.0)));

  mca::control::RobustCbfSlewRateFilter robust_slew;
  ASSERT_TRUE(robust_slew.configure(config).ok());
  mca::control::RobustCbfSlewRateInput robust_slew_input;
  robust_slew_input.robust_prediction = robust_input;
  robust_slew_input.previous_control = Eigen::Vector2d::Zero();
  robust_slew_input.max_control_delta = Eigen::Vector2d::Constant(0.1);
  EXPECT_EQ(robust_slew.filter(robust_slew_input, sentinel).code,
            mca::core::StatusCode::ConstraintViolation);
  EXPECT_TRUE(sentinel.control.isApprox(Eigen::Vector2d(7.0, 8.0)));
}
}  // namespace
