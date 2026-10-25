#include <gtest/gtest.h>

#include <limits>

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

TEST(DcMotor, InvalidReconfigurationPreservesWorkingModel) {
  mca::plant::DcMotorConfig valid_config;
  valid_config.resistance_ohm = 2.0;
  mca::plant::DcMotor motor(valid_config);
  ASSERT_TRUE(motor.configured());
  ASSERT_TRUE(motor.reset(0.25, 1.5, 0.75).ok());

  auto invalid_config = valid_config;
  invalid_config.inductance_h = 0.0;
  EXPECT_EQ(motor.configure(invalid_config).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_TRUE(motor.configured());
  EXPECT_DOUBLE_EQ(motor.config().resistance_ohm, 2.0);
  EXPECT_DOUBLE_EQ(motor.position(), 0.25);
  EXPECT_DOUBLE_EQ(motor.velocity(), 1.5);
  EXPECT_DOUBLE_EQ(motor.current(), 0.75);

  ASSERT_TRUE(motor.step(1.0, 0.0, 0.001).ok());
  EXPECT_NE(motor.current(), 0.75);
}

TEST(DcMotor, ResetTorqueOverflowPreservesState) {
  mca::plant::DcMotorConfig config;
  config.torque_constant_nm_per_a = 2.0;
  mca::plant::DcMotor motor(config);
  ASSERT_TRUE(motor.configured());
  ASSERT_TRUE(motor.reset(0.25, 1.5, 0.75).ok());
  const double position_before = motor.position();
  const double velocity_before = motor.velocity();
  const double current_before = motor.current();

  EXPECT_EQ(motor.reset(9.0, 8.0, std::numeric_limits<double>::max()).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(motor.position(), position_before);
  EXPECT_DOUBLE_EQ(motor.velocity(), velocity_before);
  EXPECT_DOUBLE_EQ(motor.current(), current_before);
  EXPECT_DOUBLE_EQ(motor.torque(), config.torque_constant_nm_per_a * current_before);

  ASSERT_TRUE(motor.reset(0.0, 0.0, 0.5).ok());
  EXPECT_DOUBLE_EQ(motor.torque(), 1.0);
  EXPECT_TRUE(motor.step(1.0, 0.0, 0.001).ok());
}

TEST(DcMotor, NumericalOverflowDoesNotCommitPartialState) {
  mca::plant::DcMotor motor;
  ASSERT_TRUE(motor.configure(mca::plant::DcMotorConfig{}).ok());
  ASSERT_TRUE(motor.reset(0.25, 1.5, 0.75).ok());
  const double position_before = motor.position();
  const double velocity_before = motor.velocity();
  const double current_before = motor.current();

  EXPECT_EQ(motor.step(std::numeric_limits<double>::max(), 0.0, 1.0).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(motor.position(), position_before);
  EXPECT_DOUBLE_EQ(motor.velocity(), velocity_before);
  EXPECT_DOUBLE_EQ(motor.current(), current_before);

  EXPECT_EQ(motor.step(1.0, std::numeric_limits<double>::max(), 1.0).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(motor.position(), position_before);
  EXPECT_DOUBLE_EQ(motor.velocity(), velocity_before);
  EXPECT_DOUBLE_EQ(motor.current(), current_before);

  EXPECT_TRUE(motor.step(1.0, 0.0, 0.001).ok());
}
TEST(DcMotor, CheckedAccelerationReportsStatusAndPreservesOutput) {
  mca::plant::DcMotor unconfigured;
  double output = 123.0;
  EXPECT_EQ(unconfigured.acceleration(0.0, output).code,
            mca::core::StatusCode::NotConfigured);
  EXPECT_DOUBLE_EQ(output, 123.0);

  mca::plant::DcMotorConfig config;
  config.torque_constant_nm_per_a = 0.5;
  config.inertia_kg_m2 = 2.0;
  config.viscous_damping_nm_per_rad_s = 0.25;
  mca::plant::DcMotor motor;
  ASSERT_TRUE(motor.configure(config).ok());
  ASSERT_TRUE(motor.reset(0.0, 4.0, 6.0).ok());

  EXPECT_EQ(motor.acceleration(std::numeric_limits<double>::infinity(), output).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_DOUBLE_EQ(output, 123.0);
  ASSERT_TRUE(motor.acceleration(0.5, output).ok());
  EXPECT_DOUBLE_EQ(output, (3.0 - 1.0 - 0.5) / 2.0);
  EXPECT_DOUBLE_EQ(motor.velocity(), 4.0);
  EXPECT_DOUBLE_EQ(motor.current(), 6.0);
}

TEST(DcMotor, CheckedAccelerationRejectsOverflowWithoutChangingOutput) {
  mca::plant::DcMotorConfig config;
  config.viscous_damping_nm_per_rad_s =
      std::numeric_limits<double>::max() / 2.0;
  mca::plant::DcMotor motor;
  ASSERT_TRUE(motor.configure(config).ok());
  ASSERT_TRUE(motor.reset(0.0, 4.0, 0.0).ok());
  double output = -77.0;
  EXPECT_EQ(motor.acceleration(0.0, output).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(output, -77.0);
  EXPECT_DOUBLE_EQ(motor.velocity(), 4.0);
  EXPECT_DOUBLE_EQ(motor.current(), 0.0);
}

TEST(DcMotor, CheckedTorqueReportsStatusAndPreservesOutput) {
  mca::plant::DcMotor unconfigured;
  double output = 123.0;
  EXPECT_EQ(unconfigured.torque(output).code,
            mca::core::StatusCode::NotConfigured);
  EXPECT_DOUBLE_EQ(output, 123.0);

  mca::plant::DcMotor motor;
  ASSERT_TRUE(motor.configure(mca::plant::DcMotorConfig{}).ok());
  ASSERT_TRUE(motor.reset(0.0, 0.0, 1.25).ok());
  ASSERT_TRUE(motor.torque(output).ok());
  EXPECT_DOUBLE_EQ(output, motor.torque());
  EXPECT_DOUBLE_EQ(output, 0.125);
}

TEST(DcMotor, CheckedMechanicalPowerReportsStatusAndPreservesOutput) {
  mca::plant::DcMotor unconfigured;
  double output = 123.0;
  EXPECT_EQ(unconfigured.mechanical_power(output).code,
            mca::core::StatusCode::NotConfigured);
  EXPECT_DOUBLE_EQ(output, 123.0);

  mca::plant::DcMotorConfig config;
  config.torque_constant_nm_per_a = 0.5;
  mca::plant::DcMotor motor(config);
  ASSERT_TRUE(motor.reset(0.25, 4.0, 6.0).ok());
  ASSERT_TRUE(motor.mechanical_power(output).ok());
  EXPECT_DOUBLE_EQ(output, 12.0);
  EXPECT_DOUBLE_EQ(motor.position(), 0.25);
  EXPECT_DOUBLE_EQ(motor.velocity(), 4.0);
  EXPECT_DOUBLE_EQ(motor.current(), 6.0);
}

TEST(DcMotor, CheckedMechanicalPowerRejectsOverflowWithoutChangingOutput) {
  mca::plant::DcMotorConfig config;
  config.torque_constant_nm_per_a = 1.0;
  mca::plant::DcMotor motor(config);
  ASSERT_TRUE(motor.reset(0.0, 2.0, std::numeric_limits<double>::max()).ok());
  double output = -77.0;
  EXPECT_EQ(motor.mechanical_power(output).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(output, -77.0);
  EXPECT_DOUBLE_EQ(motor.velocity(), 2.0);
  EXPECT_DOUBLE_EQ(motor.current(), std::numeric_limits<double>::max());
}
