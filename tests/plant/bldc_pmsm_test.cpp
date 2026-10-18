#include <gtest/gtest.h>

#include "mca/plant/bldc_pmsm.hpp"

TEST(BldcPmsm, RejectsInvalidConfiguration) {
  mca::plant::BldcPmsm motor;
  mca::plant::BldcPmsmConfig config;
  config.pole_pairs = 0;
  EXPECT_FALSE(motor.configure(config).ok());
  EXPECT_FALSE(motor.configured());
}

TEST(BldcPmsm, AppliesThreePhaseVoltageAndProducesCurrent) {
  mca::plant::BldcPmsmConfig config;
  config.phase_resistance_ohm = 1.0;
  config.phase_inductance_h = 0.01;
  config.inertia_kg_m2 = 1.0;
  config.viscous_damping_nm_per_rad_s = 1.0;
  mca::plant::BldcPmsm motor(config);
  ASSERT_TRUE(motor.configured());

  for (int i = 0; i < 100; ++i) {
    ASSERT_TRUE(motor.step(1.0, -0.5, -0.5, 0.0, 0.0001).ok());
  }

  EXPECT_GT(motor.current_a(), 0.0);
  EXPECT_LT(motor.current_b(), 0.0);
  EXPECT_LT(motor.current_c(), 0.0);
  EXPECT_GT(motor.speed(), 0.0);
}

TEST(BldcPmsm, ElectricalAngleUsesPolePairCount) {
  mca::plant::BldcPmsmConfig config;
  config.pole_pairs = 5;
  mca::plant::BldcPmsm motor(config);
  ASSERT_TRUE(motor.reset(0.2, 0.0).ok());
  EXPECT_NEAR(motor.electrical_angle(), 1.0, 1e-12);
}

TEST(BldcPmsm, RejectsNonFiniteInputAndInvalidStep) {
  mca::plant::BldcPmsm motor(mca::plant::BldcPmsmConfig{});
  EXPECT_FALSE(motor.step(0.0, 0.0, 0.0, 0.0, 0.0).ok());
  EXPECT_FALSE(motor.step(0.0, 0.0, 0.0, 0.0,
                          std::numeric_limits<double>::quiet_NaN()).ok());
}
