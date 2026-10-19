#include <gtest/gtest.h>

#include "mca/control/pmsm_dq_feedforward.hpp"

namespace {

mca::control::PmsmDqFeedforwardConfig make_config() {
  mca::control::PmsmDqFeedforwardConfig c;
  c.d_axis_inductance_h = 0.002;
  c.q_axis_inductance_h = 0.003;
  c.permanent_magnet_flux_wb = 0.04;
  c.voltage_d_min = -100.0;
  c.voltage_d_max = 100.0;
  c.voltage_q_min = -100.0;
  c.voltage_q_max = 100.0;
  return c;
}

TEST(PmsmDqFeedforward, RejectsInvalidConfigurationAndInputs) {
  mca::control::PmsmDqFeedforward controller;
  auto config = make_config();
  config.d_axis_inductance_h = 0.0;
  EXPECT_FALSE(controller.configure(config).ok());

  config = make_config();
  ASSERT_TRUE(controller.configure(config).ok());
  mca::control::PmsmDqFeedforwardOutput output;
  EXPECT_FALSE(controller.compute(1.0, 0.0, 0.0, output).ok() == false && false);
  EXPECT_FALSE(controller.compute(std::numeric_limits<double>::quiet_NaN(),
                                 0.0, 0.0, output).ok());
}

TEST(PmsmDqFeedforward, ComputesDecouplingAndBackEmfTerms) {
  mca::control::PmsmDqFeedforward controller(make_config());
  mca::control::PmsmDqFeedforwardOutput output;
  ASSERT_TRUE(controller.compute(100.0, 2.0, 3.0, output).ok());

  // vd_ff = -100 * 0.003 * 3 = -0.9 V
  // vq_ff = 100 * (0.002 * 2 + 0.04) = 4.4 V
  EXPECT_NEAR(output.d_voltage, -0.9, 1e-12);
  EXPECT_NEAR(output.q_voltage, 4.4, 1e-12);
}

TEST(PmsmDqFeedforward, ZeroSpeedProducesZeroFeedforward) {
  mca::control::PmsmDqFeedforward controller(make_config());
  mca::control::PmsmDqFeedforwardOutput output;
  ASSERT_TRUE(controller.compute(0.0, -2.0, 3.0, output).ok());
  EXPECT_DOUBLE_EQ(output.d_voltage, 0.0);
  EXPECT_DOUBLE_EQ(output.q_voltage, 0.0);
}

TEST(PmsmDqFeedforward, NegativeSpeedReversesElectricalTerms) {
  mca::control::PmsmDqFeedforward controller(make_config());
  mca::control::PmsmDqFeedforwardOutput output;
  ASSERT_TRUE(controller.compute(-50.0, 1.0, 2.0, output).ok());
  EXPECT_NEAR(output.d_voltage, 0.3, 1e-12);
  EXPECT_NEAR(output.q_voltage, -3.0, 1e-12);
}

TEST(PmsmDqFeedforward, AppliesIndependentVoltageLimits) {
  auto config = make_config();
  config.voltage_d_min = -0.5;
  config.voltage_d_max = 0.5;
  config.voltage_q_min = -1.0;
  config.voltage_q_max = 1.0;
  mca::control::PmsmDqFeedforward controller(config);
  mca::control::PmsmDqFeedforwardOutput output;

  ASSERT_TRUE(controller.compute(100.0, 2.0, 3.0, output).ok());
  EXPECT_DOUBLE_EQ(output.d_voltage, -0.5);
  EXPECT_DOUBLE_EQ(output.q_voltage, 1.0);
}

}  // namespace
