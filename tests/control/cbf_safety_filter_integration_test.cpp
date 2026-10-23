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
  prediction.predicted_drift[0] = -10.0;

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

TEST(CbfSafetyFilterIntegration, BoundaryConstraintRemainsFeasibleAcrossVariants) {
  const auto config = make_config();
  auto prediction = make_prediction();
  prediction.predicted_drift[0] = -0.5;
  prediction.nominal_control = Eigen::Vector2d(1.0, 0.0);

  mca::control::DiscreteTimeCbfFilter nominal;
  mca::control::RobustDiscreteTimeCbfFilter robust;
  mca::control::CbfSlewRateFilter slew;
  mca::control::RobustCbfSlewRateFilter robust_slew;
  ASSERT_TRUE(nominal.configure(config).ok());
  ASSERT_TRUE(robust.configure(config).ok());
  ASSERT_TRUE(slew.configure(config).ok());
  ASSERT_TRUE(robust_slew.configure(config).ok());

  mca::control::DiscreteTimeCbfOutput output;
  ASSERT_TRUE(nominal.filter(prediction, output).ok());
  EXPECT_NEAR(output.control[0], 1.0, 1.0e-8);
  EXPECT_GE(output.minimum_safety_margin, -1.0e-8);

  mca::control::RobustDiscreteTimeCbfInput robust_input;
  robust_input.prediction = prediction;
  robust_input.prediction_error_bounds = Eigen::VectorXd::Zero(1);
  ASSERT_TRUE(robust.filter(robust_input, output).ok());
  EXPECT_NEAR(output.control[0], 1.0, 1.0e-8);

  mca::control::CbfSlewRateInput slew_input;
  slew_input.prediction = prediction;
  slew_input.previous_control = Eigen::Vector2d::Zero();
  slew_input.max_control_delta = Eigen::Vector2d::Constant(10.0);
  ASSERT_TRUE(slew.filter(slew_input, output).ok());
  EXPECT_NEAR(output.control[0], 1.0, 1.0e-8);

  mca::control::RobustCbfSlewRateInput robust_slew_input;
  robust_slew_input.robust_prediction = robust_input;
  robust_slew_input.previous_control = slew_input.previous_control;
  robust_slew_input.max_control_delta = slew_input.max_control_delta;
  ASSERT_TRUE(robust_slew.filter(robust_slew_input, output).ok());
  EXPECT_NEAR(output.control[0], 1.0, 1.0e-8);
}

TEST(CbfSafetyFilterIntegration, ArithmeticOverflowReturnsNumericalErrorTransactionally) {
  const auto config = make_config();
  auto prediction = make_prediction();
  prediction.barrier_values[0] = 1.7e308;
  prediction.predicted_drift[0] = -1.7e308;
  prediction.nominal_control.setZero();

  const Eigen::Vector2d sentinel_control(7.0, 8.0);
  mca::control::DiscreteTimeCbfOutput output;
  output.control = sentinel_control;

  mca::control::DiscreteTimeCbfFilter nominal;
  ASSERT_TRUE(nominal.configure(config).ok());
  EXPECT_EQ(nominal.filter(prediction, output).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_TRUE(output.control.isApprox(sentinel_control));

  mca::control::RobustDiscreteTimeCbfFilter robust;
  ASSERT_TRUE(robust.configure(config).ok());
  mca::control::RobustDiscreteTimeCbfInput robust_input;
  robust_input.prediction = prediction;
  robust_input.prediction_error_bounds = Eigen::VectorXd::Zero(1);
  EXPECT_EQ(robust.filter(robust_input, output).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_TRUE(output.control.isApprox(sentinel_control));

  mca::control::CbfSlewRateFilter slew;
  ASSERT_TRUE(slew.configure(config).ok());
  mca::control::CbfSlewRateInput slew_input;
  slew_input.prediction = prediction;
  slew_input.previous_control = Eigen::Vector2d::Zero();
  slew_input.max_control_delta = Eigen::Vector2d::Constant(10.0);
  EXPECT_EQ(slew.filter(slew_input, output).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_TRUE(output.control.isApprox(sentinel_control));

  mca::control::RobustCbfSlewRateFilter robust_slew;
  ASSERT_TRUE(robust_slew.configure(config).ok());
  mca::control::RobustCbfSlewRateInput robust_slew_input;
  robust_slew_input.robust_prediction = robust_input;
  robust_slew_input.previous_control = slew_input.previous_control;
  robust_slew_input.max_control_delta = slew_input.max_control_delta;
  EXPECT_EQ(robust_slew.filter(robust_slew_input, output).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_TRUE(output.control.isApprox(sentinel_control));
}

TEST(CbfSafetyFilterIntegration, MultiStepClosedLoopPreservesSafetyAndSlewInvariants) {
  mca::control::DiscreteTimeCbfConfig config;
  config.gamma = 0.2;
  config.control_min = Eigen::VectorXd::Constant(1, -1.0);
  config.control_max = Eigen::VectorXd::Constant(1, 1.0);
  ASSERT_GT(config.max_total_constraints, 0U);

  mca::control::DiscreteTimeCbfFilter nominal;
  mca::control::RobustDiscreteTimeCbfFilter robust;
  mca::control::CbfSlewRateFilter slew;
  mca::control::RobustCbfSlewRateFilter robust_slew;
  ASSERT_TRUE(nominal.configure(config).ok());
  ASSERT_TRUE(robust.configure(config).ok());
  ASSERT_TRUE(slew.configure(config).ok());
  ASSERT_TRUE(robust_slew.configure(config).ok());

  const Eigen::VectorXd rate = Eigen::VectorXd::Constant(1, 0.25);
  const double prediction_error = -0.05;

  auto run = [&](auto& filter, bool use_robust, bool use_slew) {
    double barrier = 1.0;
    Eigen::VectorXd previous = Eigen::VectorXd::Zero(1);
    std::vector<double> controls;
    std::vector<double> barriers;
    std::vector<bool> modified;
    controls.reserve(12);
    barriers.reserve(12);
    modified.reserve(12);

    for (int k = 0; k < 12; ++k) {
      mca::control::DiscreteTimeCbfInput prediction;
      prediction.barrier_values = Eigen::VectorXd::Constant(1, barrier);
      prediction.predicted_drift = Eigen::VectorXd::Constant(1, barrier);
      prediction.control_derivatives = Eigen::MatrixXd::Ones(1, 1);
      prediction.nominal_control = Eigen::VectorXd::Constant(1, (k % 2 == 0) ? -1.0 : 0.5);

      mca::control::DiscreteTimeCbfOutput output;
      if (use_robust && use_slew) {
        mca::control::RobustCbfSlewRateInput input;
        input.robust_prediction.prediction = prediction;
        input.robust_prediction.prediction_error_bounds = Eigen::VectorXd::Constant(1, 0.05);
        input.previous_control = previous;
        input.max_control_delta = rate;
        ASSERT_TRUE(filter.filter(input, output).ok());
      } else if (use_robust) {
        mca::control::RobustDiscreteTimeCbfInput input;
        input.prediction = prediction;
        input.prediction_error_bounds = Eigen::VectorXd::Constant(1, 0.05);
        ASSERT_TRUE(filter.filter(input, output).ok());
      } else if (use_slew) {
        mca::control::CbfSlewRateInput input;
        input.prediction = prediction;
        input.previous_control = previous;
        input.max_control_delta = rate;
        ASSERT_TRUE(filter.filter(input, output).ok());
      } else {
        ASSERT_TRUE(filter.filter(prediction, output).ok());
      }

      ASSERT_TRUE(output.control.allFinite());
      EXPECT_GE(output.control[0], -1.0 - 1.0e-9);
      EXPECT_LE(output.control[0], 1.0 + 1.0e-9);
      if (use_slew) {
        EXPECT_LE(std::abs(output.control[0] - previous[0]), 0.25 + 1.0e-9);
      }

      const double error = use_robust ? prediction_error : 0.0;
      const double next_barrier = barrier + output.control[0] + error;
      EXPECT_GE(next_barrier, 0.8 * barrier - 2.0e-8);
      EXPECT_GT(next_barrier, 0.0);

      barrier = next_barrier;
      previous = output.control;
      controls.push_back(output.control[0]);
      barriers.push_back(barrier);
      modified.push_back(output.modified);
    }

    EXPECT_GT(barriers.back(), 0.0);
    return std::make_pair(controls, modified);
  };

  const auto nominal_trace = run(nominal, false, false);
  const auto robust_trace = run(robust, true, false);
  const auto slew_trace = run(slew, false, true);
  const auto robust_slew_trace = run(robust_slew, true, true);

  ASSERT_EQ(nominal_trace.first.size(), 12U);
  ASSERT_EQ(robust_trace.first.size(), 12U);
  ASSERT_EQ(slew_trace.first.size(), 12U);
  ASSERT_EQ(robust_slew_trace.first.size(), 12U);
  EXPECT_TRUE(robust_trace.second.front());
  EXPECT_TRUE(slew_trace.second.front());
  EXPECT_TRUE(robust_slew_trace.second.front());

  const auto robust_slew_trace_repeat = run(robust_slew, true, true);
  ASSERT_EQ(robust_slew_trace_repeat.first.size(), robust_slew_trace.first.size());
  for (std::size_t i = 0; i < robust_slew_trace.first.size(); ++i) {
    EXPECT_DOUBLE_EQ(robust_slew_trace_repeat.first[i], robust_slew_trace.first[i]);
    EXPECT_EQ(robust_slew_trace_repeat.second[i], robust_slew_trace.second[i]);
  }
}
}  // namespace
