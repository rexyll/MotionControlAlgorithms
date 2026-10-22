#include <cmath>
#include <Eigen/Core>
#include <gtest/gtest.h>
#include "mca/control/robust_adaptive.hpp"

namespace {
using namespace mca::control;

TEST(RobustAdaptive, AppliesAdaptiveAndContinuousRobustTerms) {
  RobustAdaptiveController controller;
  RobustAdaptiveConfig config;
  config.adaptation_gain = 2.0;
  config.robust_gain = 0.4;
  ASSERT_TRUE(controller.configure(config, 2));
  RobustAdaptiveInput input;
  input.regressor = Eigen::Vector2d(1.0, 0.5);
  input.sliding_error = 0.01;
  input.nominal_control = 1.0;
  input.dt = 0.01;
  RobustAdaptiveOutput output;
  ASSERT_TRUE(controller.update(input, output));
  EXPECT_NEAR(output.control, 1.0 + 0.0 - 0.2, 1e-12);
  EXPECT_NEAR(output.robust_term, -0.2, 1e-12);
  EXPECT_GT(controller.parameters().norm(), 0.0);
}

TEST(RobustAdaptive, ProjectsParametersAndLimitsControl) {
  RobustAdaptiveController controller;
  RobustAdaptiveConfig config;
  config.parameter_limit = 0.1;
  config.control_limit = 0.25;
  config.adaptation_gain = 100.0;
  ASSERT_TRUE(controller.configure(config, 1));
  RobustAdaptiveInput input;
  input.regressor = Eigen::VectorXd::Constant(1, 10.0);
  input.sliding_error = 1.0;
  input.nominal_control = 10.0;
  input.dt = 1.0;
  RobustAdaptiveOutput output;
  ASSERT_TRUE(controller.update(input, output));
  EXPECT_LE(std::abs(output.control), 0.25);
  EXPECT_LE(std::abs(controller.parameters()(0)), 0.1);
}

TEST(RobustAdaptive, RejectsInvalidInputWithoutMutatingStateOrOutput) {
  RobustAdaptiveController controller;
  ASSERT_TRUE(controller.configure(RobustAdaptiveConfig{}, 1));
  RobustAdaptiveInput input;
  input.regressor = Eigen::VectorXd::Constant(1, 1.0);
  input.sliding_error = 0.2;
  input.dt = 0.01;
  RobustAdaptiveOutput output;
  output.control = 42.0;
  ASSERT_TRUE(controller.update(input, output));
  const auto previous_parameters = controller.parameters();
  const double previous_control = output.control;
  input.dt = 0.0;
  EXPECT_FALSE(controller.update(input, output));
  EXPECT_DOUBLE_EQ(output.control, previous_control);
  EXPECT_TRUE(controller.parameters().isApprox(previous_parameters));
}

TEST(RobustAdaptive, RejectsInvalidConfigurationAndResetDimensions) {
  RobustAdaptiveController controller;
  RobustAdaptiveConfig invalid;
  invalid.boundary_layer = 0.0;
  EXPECT_FALSE(controller.configure(invalid, 1));
  ASSERT_TRUE(controller.configure(RobustAdaptiveConfig{}, 2));
  EXPECT_FALSE(controller.reset(Eigen::VectorXd::Zero(1)));
}
}  // namespace
