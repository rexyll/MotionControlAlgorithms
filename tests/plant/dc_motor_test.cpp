#include <gtest/gtest.h>

#include "mca/plant/dc_motor.hpp"

TEST(DcMotor, RejectsInvalidConfiguration) {
  mca::plant::DcMotor motor;
  mca::plant::DcMotorConfig config;
  config.resistance_ohm = 0.0;
  EXPECT_FALSE(motor.configure(config).ok());
  EXPECT_FALSE(motor.configured());
}

TEST(DcMotor, CurrentApproachesOhmicSteadyStateAtZeroSpeed) {
  mca::plant::DcMotorConfig config;
  config.resistance_ohm = 2.0;
  config.inductance_h = 0.01;
  config.torque_constant_nm_per_a = 0.1;
  config.back_emf_constant_v_per_rad_s = 0.1;
  config.inertia_kg_m2 = 1.0;
  config.viscous_damping_nm_per_rad_s = 1.0;

  mca::plant::DcMotor motor(config);
  ASSERT_TRUE(motor.configured());
  ASSERT_TRUE(motor.reset(0.0, 0.0, 0.0).ok());

  for (int i = 0; i < 1000; ++i) {
    ASSERT_TRUE(motor.step(2.0, 0.0, 0.0001).ok());
  }

  EXPECT_NEAR(motor.current(), 1.0, 1e-3);
}

TEST(DcMotor, GeneratesPositiveTorqueAndMotion) {
  mca::plant::DcMotorConfig config;
  config.resistance_ohm = 1.0;
  config.inductance_h = 0.001;
  config.torque_constant_nm_per_a = 0.2;
  config.back_emf_constant_v_per_rad_s = 0.2;
  config.inertia_kg_m2 = 0.01;
  config.viscous_damping_nm_per_rad_s = 0.001;

  mca::plant::DcMotor motor(config);
  ASSERT_TRUE(motor.configured());

  for (int i = 0; i < 2000; ++i) {
    ASSERT_TRUE(motor.step(3.0, 0.0, 0.0001).ok());
  }

  EXPECT_GT(motor.current(), 0.0);
  EXPECT_GT(motor.torque(), 0.0);
  EXPECT_GT(motor.velocity(), 0.0);
  EXPECT_GT(motor.position(), 0.0);
}

TEST(DcMotor, BackEmfReducesCurrentAtSpeed) {
  mca::plant::DcMotorConfig config;
  config.resistance_ohm = 1.0;
  config.inductance_h = 0.001;
  config.torque_constant_nm_per_a = 0.1;
  config.back_emf_constant_v_per_rad_s = 0.1;
  config.inertia_kg_m2 = 0.01;
  config.viscous_damping_nm_per_rad_s = 0.001;

  mca::plant::DcMotor motor(config);
  ASSERT_TRUE(motor.reset(0.0, 20.0, 0.0).ok());
  for (int i = 0; i < 500; ++i) {
    ASSERT_TRUE(motor.step(2.0, 0.0, 0.0001).ok());
  }

  EXPECT_LT(motor.current(), 2.0);
}
