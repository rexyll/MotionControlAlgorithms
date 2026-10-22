#include <limits>

#include <gtest/gtest.h>

#include "mca/control/control_barrier_filter.hpp"

namespace {

mca::control::ControlBarrierFilter make_filter() {
  mca::control::ControlBarrierFilter filter;
  mca::control::ControlBarrierFilterConfig config;
  config.alpha = 2.0;
  config.control_min = -1.0;
  config.control_max = 1.0;
  config.coefficient_tolerance = 1.0e-9;
  EXPECT_TRUE(filter.configure(config).ok());
  return filter;
}

TEST(ControlBarrierFilter, ProjectsNominalCommandToSafeBoundary) {
  auto filter = make_filter();
  mca::control::ControlBarrierFilterInput input;
  input.barrier_value = -0.2;
  input.drift_derivative = 0.0;
  input.control_derivative = 1.0;
  input.nominal_control = -0.9;
  mca::control::ControlBarrierFilterOutput output;
  ASSERT_TRUE(filter.filter(input, output).ok());
  EXPECT_NEAR(output.control, 0.4, 1.0e-12);
  EXPECT_NEAR(output.safety_margin, 0.0, 1.0e-12);
  EXPECT_TRUE(output.modified);
}

TEST(ControlBarrierFilter, HandlesNegativeControlEffectAndActuatorBounds) {
  auto filter = make_filter();
  mca::control::ControlBarrierFilterInput input;
  input.barrier_value = -0.1;
  input.control_derivative = -1.0;
  input.nominal_control = 0.9;
  mca::control::ControlBarrierFilterOutput output;
  ASSERT_TRUE(filter.filter(input, output).ok());
  EXPECT_NEAR(output.control, -0.2, 1.0e-12);
  EXPECT_GE(output.safety_margin, -1.0e-9);
}

TEST(ControlBarrierFilter, LeavesSafeNominalCommandUnchanged) {
  auto filter = make_filter();
  mca::control::ControlBarrierFilterInput input;
  input.barrier_value = 1.0;
  input.drift_derivative = -0.2;
  input.control_derivative = 0.5;
  input.nominal_control = 0.3;
  mca::control::ControlBarrierFilterOutput output;
  ASSERT_TRUE(filter.filter(input, output).ok());
  EXPECT_DOUBLE_EQ(output.control, input.nominal_control);
  EXPECT_FALSE(output.modified);
  EXPECT_GT(output.safety_margin, 0.0);
}

TEST(ControlBarrierFilter, ReportsInfeasibleSafetyConstraint) {
  auto filter = make_filter();
  mca::control::ControlBarrierFilterInput input;
  input.barrier_value = -1.0;
  input.drift_derivative = 0.0;
  input.control_derivative = 0.0;
  input.nominal_control = 0.0;
  mca::control::ControlBarrierFilterOutput output;
  output.control = 17.0;
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::ConstraintViolation);
  EXPECT_DOUBLE_EQ(output.control, 17.0);
}

TEST(ControlBarrierFilter, RejectsInvalidInputAndConfigurationTransactionally) {
  mca::control::ControlBarrierFilter filter;
  mca::control::ControlBarrierFilterConfig bad;
  bad.alpha = 0.0;
  EXPECT_FALSE(filter.configure(bad).ok());
  EXPECT_EQ(filter.filter({}, *new mca::control::ControlBarrierFilterOutput()).code,
            mca::core::StatusCode::NotConfigured);
  ASSERT_TRUE(filter.configure({}).ok());
  mca::control::ControlBarrierFilterInput input;
  input.nominal_control = std::numeric_limits<double>::quiet_NaN();
  mca::control::ControlBarrierFilterOutput output;
  output.control = 42.0;
  EXPECT_EQ(filter.filter(input, output).code, mca::core::StatusCode::InvalidArgument);
  EXPECT_DOUBLE_EQ(output.control, 42.0);
}

}  // namespace
