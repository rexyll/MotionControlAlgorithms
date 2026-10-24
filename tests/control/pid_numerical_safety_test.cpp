#include <limits>

#include <gtest/gtest.h>

#include "mca/control/pid.hpp"

TEST(PidNumericalSafety, ProportionalOverflowIsTransactional) {
  mca::control::PidConfig config;
  config.kp = std::numeric_limits<double>::max();
  mca::control::PidController pid(config);
  double output = 0.0;

  ASSERT_TRUE(pid.update(1.0, 0.0, 0.01, output).ok());
  const double previous_output = output;
  const double previous_integral = pid.integral();

  const auto status = pid.update(2.0, -2.0, 0.01, output);
  EXPECT_EQ(status.code, mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(output, previous_output);
  EXPECT_DOUBLE_EQ(pid.output(), previous_output);
  EXPECT_DOUBLE_EQ(pid.integral(), previous_integral);
}

TEST(PidNumericalSafety, IntegralOverflowIsTransactional) {
  mca::control::PidConfig config;
  config.ki = std::numeric_limits<double>::max();
  config.output_min = -std::numeric_limits<double>::max();
  config.output_max = std::numeric_limits<double>::max();
  config.integral_min = -std::numeric_limits<double>::max();
  config.integral_max = std::numeric_limits<double>::max();
  mca::control::PidController pid(config);
  double output = 0.0;

  ASSERT_TRUE(pid.update(1.0, 0.0, 0.5, output).ok());
  const double previous_output = output;
  const double previous_integral = pid.integral();

  const auto status = pid.update(2.0, 0.0, 2.0, output);
  EXPECT_EQ(status.code, mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(output, previous_output);
  EXPECT_DOUBLE_EQ(pid.output(), previous_output);
  EXPECT_DOUBLE_EQ(pid.integral(), previous_integral);
}

TEST(PidNumericalSafety, DerivativeOverflowIsTransactional) {
  mca::control::PidConfig config;
  config.kd = 1.0;
  config.output_min = -std::numeric_limits<double>::max();
  config.output_max = std::numeric_limits<double>::max();
  mca::control::PidController pid(config);
  double output = 0.0;
  ASSERT_TRUE(pid.reset(0.0, 0.0).ok());
  ASSERT_TRUE(pid.update(0.0, 1.0, 1.0, output).ok());
  const double previous_output = output;
  const double previous_integral = pid.integral();

  const auto status = pid.update(0.0, -1.0,
                                 std::numeric_limits<double>::denorm_min(),
                                 output);
  EXPECT_EQ(status.code, mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(output, previous_output);
  EXPECT_DOUBLE_EQ(pid.output(), previous_output);
  EXPECT_DOUBLE_EQ(pid.integral(), previous_integral);
}

TEST(PidNumericalSafety, FirstUpdateOverflowDoesNotInitializeState) {
  mca::control::PidConfig config;
  config.kp = std::numeric_limits<double>::max();
  mca::control::PidController pid(config);
  double output = 7.0;

  const auto status = pid.update(2.0, -2.0, 0.01, output);
  EXPECT_EQ(status.code, mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(output, 7.0);
  EXPECT_DOUBLE_EQ(pid.output(), 0.0);
  EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
}
