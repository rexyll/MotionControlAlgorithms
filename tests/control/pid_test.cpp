#include <gtest/gtest.h>

#include "mca/control/pid.hpp"

TEST(PidController, ProportionalResponse) {
  mca::control::PidConfig config;
  config.kp = 2.0;
  ASSERT_TRUE(mca::control::PidController(config).configured());

  mca::control::PidController pid(config);
  double output = 0.0;
  ASSERT_TRUE(pid.update(3.0, 1.0, 0.001, output).ok());
  EXPECT_DOUBLE_EQ(output, 4.0);
}

TEST(PidController, IntegralAccumulates) {
  mca::control::PidConfig config;
  config.ki = 2.0;
  mca::control::PidController pid(config);
  double output = 0.0;

  ASSERT_TRUE(pid.update(1.0, 0.0, 0.1, output).ok());
  EXPECT_NEAR(output, 0.2, 1e-12);
  ASSERT_TRUE(pid.update(1.0, 0.0, 0.1, output).ok());
  EXPECT_NEAR(output, 0.4, 1e-12);
}

TEST(PidController, OutputSaturationPreventsWindup) {
  mca::control::PidConfig config;
  config.ki = 10.0;
  config.output_min = -1.0;
  config.output_max = 1.0;
  config.integral_min = -100.0;
  config.integral_max = 100.0;
  mca::control::PidController pid(config);
  double output = 0.0;

  for (int i = 0; i < 100; ++i) {
    ASSERT_TRUE(pid.update(10.0, 0.0, 0.1, output).ok());
    EXPECT_DOUBLE_EQ(output, 1.0);
  }
  EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
}

TEST(PidController, DerivativeUsesMeasurement) {
  mca::control::PidConfig config;
  config.kd = 2.0;
  mca::control::PidController pid(config);
  double output = 0.0;

  ASSERT_TRUE(pid.update(0.0, 0.0, 0.1, output).ok());
  ASSERT_TRUE(pid.update(0.0, 0.5, 0.1, output).ok());
  EXPECT_NEAR(output, -10.0, 1e-12);
}
