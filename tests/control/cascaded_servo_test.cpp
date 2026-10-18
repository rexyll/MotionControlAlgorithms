#include <cmath>

#include <gtest/gtest.h>

#include "mca/control/cascaded_servo.hpp"
#include "mca/plant/dc_motor.hpp"

namespace {

mca::control::CascadedServoConfig make_config() {
  mca::control::CascadedServoConfig config;
  config.position.kp = 10.0;
  config.position.output_min = -20.0;
  config.position.output_max = 20.0;

  config.velocity.kp = 1.0;
  config.velocity.output_min = -0.5;
  config.velocity.output_max = 0.5;

  config.torque.kp = 5.0;
  config.torque.output_min = -12.0;
  config.torque.output_max = 12.0;
  return config;
}

}  // namespace

TEST(CascadedServoTest, RejectsInvalidConfiguration) {
  auto config = make_config();
  config.velocity.output_min = 1.0;
  config.velocity.output_max = -1.0;

  mca::control::CascadedServo servo;
  EXPECT_FALSE(servo.configure(config).ok());
  EXPECT_FALSE(servo.configured());
}

TEST(CascadedServoTest, TracksPositionThroughDcMotor) {
  mca::control::CascadedServo servo(make_config());
  ASSERT_TRUE(servo.configured());

  mca::plant::DcMotorConfig motor_config;
  motor_config.resistance_ohm = 1.0;
  motor_config.inductance_h = 0.01;
  motor_config.torque_constant_nm_per_a = 0.1;
  motor_config.back_emf_constant_v_per_rad_s = 0.1;
  motor_config.inertia_kg_m2 = 0.01;
  motor_config.viscous_damping_nm_per_rad_s = 0.001;
  mca::plant::DcMotor motor(motor_config);

  constexpr double dt = 1.0e-4;
  constexpr int steps = 10000;
  double voltage = 0.0;
  for (int step = 0; step < steps; ++step) {
    ASSERT_TRUE(servo.update(1.0, motor.position(), motor.velocity(), motor.torque(),
                             dt, voltage)
                    .ok());
    ASSERT_TRUE(motor.step(voltage, 0.0, dt).ok());
  }

  EXPECT_NEAR(motor.position(), 1.0, 0.01);
  EXPECT_LT(std::abs(motor.velocity()), 0.05);
  EXPECT_LT(std::abs(motor.torque()), 0.01);
  EXPECT_LE(std::abs(voltage), 12.0);
  EXPECT_NEAR(servo.velocity_reference(), 0.0, 0.05);
  EXPECT_NEAR(servo.torque_reference(), 0.0, 0.05);
}

TEST(CascadedServoTest, ExposesIntermediateReferences) {
  mca::control::CascadedServo servo(make_config());
  ASSERT_TRUE(servo.reset().ok());

  double voltage = 0.0;
  ASSERT_TRUE(servo.update(0.5, 0.0, 0.0, 0.0, 0.001, voltage).ok());

  EXPECT_DOUBLE_EQ(servo.velocity_reference(), 5.0);
  EXPECT_DOUBLE_EQ(servo.torque_reference(), 0.5);
  EXPECT_DOUBLE_EQ(voltage, 2.5);
  EXPECT_DOUBLE_EQ(servo.actuator_command(), voltage);
}
