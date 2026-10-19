#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "mca/control/foc_current.hpp"
#include "mca/plant/pmsm_dq.hpp"

namespace {

TEST(FocCurrentClosedLoop, TracksDqCurrentAtLowSpeed) {
  mca::plant::PmsmDqConfig plant_config;
  plant_config.resistance_ohm = 0.5;
  plant_config.d_axis_inductance_h = 0.001;
  plant_config.q_axis_inductance_h = 0.001;
  plant_config.permanent_magnet_flux_wb = 0.05;
  // Keep the mechanical state effectively at low speed so this acceptance
  // test isolates the current-loop behavior. Speed-dependent decoupling is a
  // separate P1-09 concern.
  plant_config.inertia_kg_m2 = 1.0;
  plant_config.viscous_damping_nm_per_rad_s = 1.0;
  plant_config.pole_pairs = 2;
  mca::plant::PmsmDq motor(plant_config);
  ASSERT_TRUE(motor.reset(0.0, 0.0, 0.0, 0.0).ok());

  mca::control::FocCurrentControllerConfig controller_config;
  controller_config.kp_d = 0.8;
  controller_config.ki_d = 200.0;
  controller_config.kp_q = 0.8;
  controller_config.ki_q = 200.0;
  controller_config.current_min = -5.0;
  controller_config.current_max = 5.0;
  controller_config.voltage_d_min = -24.0;
  controller_config.voltage_d_max = 24.0;
  controller_config.voltage_q_min = -24.0;
  controller_config.voltage_q_max = 24.0;
  controller_config.integral_d_min = -24.0;
  controller_config.integral_d_max = 24.0;
  controller_config.integral_q_min = -24.0;
  controller_config.integral_q_max = 24.0;
  mca::control::FocCurrentController controller(controller_config);

  constexpr double dt = 1.0e-4;
  mca::control::FocCurrentControllerOutput command;
  double max_d_error = 0.0;
  double max_q_error = 0.0;
  for (int k = 0; k < 1000; ++k) {
    ASSERT_TRUE(controller.update(0.5, 2.0, motor.d_current(), motor.q_current(), dt,
                                  command).ok());
    ASSERT_TRUE(motor.step(command.d_voltage, command.q_voltage, 0.0, dt).ok());
    if (k >= 700) {
      max_d_error = std::max(max_d_error, std::abs(motor.d_current() - 0.5));
      max_q_error = std::max(max_q_error, std::abs(motor.q_current() - 2.0));
    }
  }

  EXPECT_LT(max_d_error, 0.02);
  EXPECT_LT(max_q_error, 0.02);
  EXPECT_LT(std::abs(motor.velocity()), 0.5);
}

}  // namespace
