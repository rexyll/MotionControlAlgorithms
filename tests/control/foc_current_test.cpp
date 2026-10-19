#include <gtest/gtest.h>

#include "mca/control/foc_current.hpp"

namespace {

mca::control::FocCurrentControllerConfig make_config() {
  mca::control::FocCurrentControllerConfig c;
  c.kp_d = 2.0;
  c.ki_d = 10.0;
  c.kp_q = 3.0;
  c.ki_q = 20.0;
  c.current_min = -5.0;
  c.current_max = 5.0;
  c.voltage_d_min = -10.0;
  c.voltage_d_max = 10.0;
  c.voltage_q_min = -10.0;
  c.voltage_q_max = 10.0;
  c.integral_d_min = -20.0;
  c.integral_d_max = 20.0;
  c.integral_q_min = -20.0;
  c.integral_q_max = 20.0;
  return c;
}

TEST(FocCurrentController, RejectsInvalidConfigurationAndDt) {
  mca::control::FocCurrentController controller;
  auto config = make_config();
  config.voltage_d_min = 2.0;
  config.voltage_d_max = 1.0;
  EXPECT_FALSE(controller.configure(config).ok());

  config = make_config();
  ASSERT_TRUE(controller.configure(config).ok());
  mca::control::FocCurrentControllerOutput output;
  EXPECT_FALSE(controller.update(0.0, 0.0, 0.0, 0.0, 0.0, output).ok());
}

TEST(FocCurrentController, PIOutputMatchesDiscreteReference) {
  mca::control::FocCurrentController controller(make_config());
  mca::control::FocCurrentControllerOutput output;
  ASSERT_TRUE(controller.update(2.0, -1.0, 0.5, -0.5, 0.01, output).ok());

  // d: e=1.5 => 2*1.5 + 10*1.5*0.01 = 3.15 V
  // q: e=-0.5 => 3*(-0.5) + 20*(-0.5)*0.01 = -1.6 V
  EXPECT_DOUBLE_EQ(output.d_reference, 2.0);
  EXPECT_DOUBLE_EQ(output.q_reference, -1.0);
  EXPECT_NEAR(output.d_voltage, 3.15, 1e-12);
  EXPECT_NEAR(output.q_voltage, -1.6, 1e-12);
}

TEST(FocCurrentController, CurrentReferencesAreClamped) {
  mca::control::FocCurrentController controller(make_config());
  mca::control::FocCurrentControllerOutput output;
  ASSERT_TRUE(controller.update(20.0, -20.0, 0.0, 0.0, 0.01, output).ok());
  EXPECT_DOUBLE_EQ(output.d_reference, 5.0);
  EXPECT_DOUBLE_EQ(output.q_reference, -5.0);
}

TEST(FocCurrentController, VoltageSaturationPreventsWindup) {
  auto config = make_config();
  config.kp_d = 0.0;
  config.ki_d = 100.0;
  config.voltage_d_max = 1.0;
  config.voltage_d_min = -1.0;
  config.integral_d_min = -100.0;
  config.integral_d_max = 100.0;
  mca::control::FocCurrentController controller(config);
  mca::control::FocCurrentControllerOutput output;

  ASSERT_TRUE(controller.update(5.0, 0.0, 0.0, 0.0, 0.01, output).ok());
  EXPECT_DOUBLE_EQ(output.d_voltage, 1.0);
  EXPECT_DOUBLE_EQ(controller.d_integral(), 0.0);

  // Reversing the error while the output is already at the negative limit
  // must not integrate further into saturation. This is the conditional
  // anti-windup rule: integration is allowed only when it moves the
  // unsaturated output back toward the valid voltage range.
  ASSERT_TRUE(controller.update(-5.0, 0.0, 0.0, 0.0, 0.01, output).ok());
  EXPECT_DOUBLE_EQ(output.d_voltage, -1.0);
  EXPECT_DOUBLE_EQ(controller.d_integral(), 0.0);
}

TEST(FocCurrentController, IntegralRecoversAfterLeavingSaturation) {
  auto config = make_config();
  config.kp_d = 0.0;
  config.ki_d = 10.0;
  config.voltage_d_max = 1.0;
  config.voltage_d_min = -1.0;
  mca::control::FocCurrentController controller(config);
  mca::control::FocCurrentControllerOutput output;

  // Start with an existing positive integral. A positive error would push
  // the requested voltage above the positive limit, so conditional
  // anti-windup must hold the integral at 0.95.
  ASSERT_TRUE(controller.reset(0.95, 0.0).ok());
  ASSERT_TRUE(controller.update(0.10, 0.0, 0.0, 0.0, 0.1, output).ok());
  EXPECT_NEAR(controller.d_integral(), 0.95, 1e-12);
  EXPECT_NEAR(output.d_voltage, 1.0, 1e-12);

  // Reverse the error. The candidate integral moves back inside the voltage
  // range, so the controller is allowed to leave saturation and recover.
  ASSERT_TRUE(controller.update(-0.10, 0.0, 0.0, 0.0, 0.1, output).ok());
  EXPECT_NEAR(controller.d_integral(), 0.85, 1e-12);
  EXPECT_NEAR(output.d_voltage, 0.85, 1e-12);
}

}  // namespace
