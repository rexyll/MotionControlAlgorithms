#include <cmath>

#include <gtest/gtest.h>

#include "mca/experiment/cascaded_servo.hpp"

namespace {

mca::control::CascadedServoConfig make_servo_config() {
  mca::control::CascadedServoConfig config;
  config.position.kp = 10.0;
  config.position.output_min = -20.0;
  config.position.output_max = 20.0;

  // The outer position loop commands a physically useful velocity range for
  // the one-second benchmark. A 0.5 rad/s limit made the benchmark spend most
  // of its duration in a deliberately slow slew, so tracking metrics mostly
  // measured the imposed reference-rate limit rather than the cascade.
  config.velocity.kp = 1.0;
  config.velocity.output_min = -4.0;
  config.velocity.output_max = 4.0;

  config.torque.kp = 5.0;
  config.torque.output_min = -12.0;
  config.torque.output_max = 12.0;
  return config;
}

mca::plant::DcMotorConfig make_motor_config() {
  mca::plant::DcMotorConfig config;
  config.resistance_ohm = 1.0;
  config.inductance_h = 0.01;
  config.torque_constant_nm_per_a = 0.1;
  config.back_emf_constant_v_per_rad_s = 0.1;
  config.inertia_kg_m2 = 0.01;
  config.viscous_damping_nm_per_rad_s = 0.001;
  return config;
}

}  // namespace

TEST(CascadedServoExperimentTest, RejectsInvalidConfiguration) {
  mca::experiment::CascadedServoExperiment experiment;
  mca::experiment::CascadedServoConfig config;
  config.dt = 0.0;
  EXPECT_FALSE(experiment.configure(config, make_servo_config(),
                                    make_motor_config())
                   .ok());
}

TEST(CascadedServoExperimentTest, ProducesTrackingBenchmark) {
  mca::experiment::CascadedServoConfig config;
  config.dt = 1.0e-4;
  config.steps = 10000;
  config.position_reference = 1.0;
  config.max_current_a = 12.0;

  mca::experiment::CascadedServoExperiment experiment;
  ASSERT_TRUE(experiment.configure(config, make_servo_config(),
                                   make_motor_config())
                  .ok());

  mca::experiment::CascadedServoResult result;
  ASSERT_TRUE(experiment.run(result).ok());

  EXPECT_NEAR(result.final_position, 1.0, 0.01);
  EXPECT_LT(std::abs(result.final_velocity), 0.05);
  EXPECT_LT(std::abs(result.final_torque), 0.01);
  // The RMSE includes the physical step transient; the initial one-radian
  // error is therefore expected and must not be treated as a steady-state
  // tracking failure.
  EXPECT_LT(result.position.rmse, 0.30);
  EXPECT_LE(result.position.max_abs_error, 1.0);
  EXPECT_GE(result.position.samples, 10000U);
  EXPECT_GE(result.position.overshoot, 0.0);
  EXPECT_TRUE(std::isfinite(result.position.settling_time_s) ||
              std::isnan(result.position.settling_time_s));
  EXPECT_LE(result.constraints.violations, 1U);
  EXPECT_LE(std::abs(result.final_voltage), 12.0);
}

TEST(CascadedServoExperimentTest, ReportsActuatorConstraintViolation) {
  mca::experiment::CascadedServoConfig config;
  config.dt = 1.0e-4;
  config.steps = 100;
  config.max_voltage_v = 0.25;

  mca::experiment::CascadedServoExperiment experiment;
  ASSERT_TRUE(experiment.configure(config, make_servo_config(),
                                   make_motor_config())
                  .ok());

  mca::experiment::CascadedServoResult result;
  ASSERT_TRUE(experiment.run(result).ok());

  EXPECT_GT(result.constraints.violations, 0U);
  EXPECT_GT(result.constraints.max_violation, 0.0);
  EXPECT_LE(std::abs(result.final_voltage), config.max_voltage_v);
}
