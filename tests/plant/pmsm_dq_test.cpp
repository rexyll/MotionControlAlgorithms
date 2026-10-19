#include <gtest/gtest.h>

#include "mca/plant/pmsm_dq.hpp"

namespace {

TEST(PmsmDq, RejectsInvalidConfigurationAndStepArguments) {
  mca::plant::PmsmDq motor;
  mca::plant::PmsmDqConfig config;
  config.pole_pairs = 0;
  EXPECT_FALSE(motor.configure(config).ok());
  EXPECT_FALSE(motor.step(0.0, 0.0, 0.0, 0.001).ok());

  config.pole_pairs = 4;
  ASSERT_TRUE(motor.configure(config).ok());
  EXPECT_FALSE(motor.step(0.0, 0.0, 0.0, 0.0).ok());
}

TEST(PmsmDq, ZeroSpeedDqVoltageMatchesElectricalEquations) {
  mca::plant::PmsmDqConfig config;
  config.resistance_ohm = 1.0;
  config.d_axis_inductance_h = 0.002;
  config.q_axis_inductance_h = 0.003;
  config.permanent_magnet_flux_wb = 0.1;
  config.inertia_kg_m2 = 0.01;
  config.viscous_damping_nm_per_rad_s = 0.0;
  config.pole_pairs = 2;

  mca::plant::PmsmDq motor(config);
  ASSERT_TRUE(motor.reset(0.0, 0.0, 2.0, 3.0).ok());
  ASSERT_TRUE(motor.step(10.0, 11.0, 0.0, 0.001).ok());

  // At zero speed: did/dt=(vd-Rid)/Ld=4000 A/s and
  // diq/dt=(vq-Riq)/Lq=2666.666... A/s.
  EXPECT_NEAR(motor.d_current(), 6.0, 1e-12);
  EXPECT_NEAR(motor.q_current(), 5.666666666666667, 1e-12);
}

TEST(PmsmDq, CrossCouplingAndBackEmfTermsHaveExpectedSigns) {
  mca::plant::PmsmDqConfig config;
  config.resistance_ohm = 1.0;
  config.d_axis_inductance_h = 0.002;
  config.q_axis_inductance_h = 0.003;
  config.permanent_magnet_flux_wb = 0.1;
  config.inertia_kg_m2 = 0.01;
  config.viscous_damping_nm_per_rad_s = 0.0;
  config.pole_pairs = 2;

  mca::plant::PmsmDq motor(config);
  ASSERT_TRUE(motor.reset(0.0, 10.0, 2.0, 3.0).ok());
  ASSERT_TRUE(motor.step(0.0, 0.0, 0.0, 0.0001).ok());

  // we=20 rad/s. Expected:
  // did/dt=(-2 + 20*0.003*3)/0.002 = 80 A/s
  // diq/dt=(-3 - 20*(0.002*2+0.1))/0.003 = -1533.333... A/s
  EXPECT_NEAR(motor.d_current(), 2.008, 1e-12);
  EXPECT_NEAR(motor.q_current(), 2.846666666666667, 1e-12);
}

TEST(PmsmDq, TorqueEquationMatchesReferenceForSurfaceMountedPmsm) {
  mca::plant::PmsmDqConfig config;
  config.permanent_magnet_flux_wb = 0.08;
  config.d_axis_inductance_h = 0.001;
  config.q_axis_inductance_h = 0.001;
  config.pole_pairs = 3;

  mca::plant::PmsmDq motor(config);
  ASSERT_TRUE(motor.reset(0.0, 0.0, -2.0, 5.0).ok());
  EXPECT_NEAR(motor.electromagnetic_torque(), 1.8, 1e-12);
}

TEST(PmsmDq, SaliencyTorqueTermIsIncluded) {
  mca::plant::PmsmDqConfig config;
  config.permanent_magnet_flux_wb = 0.0 + 0.05;
  config.d_axis_inductance_h = 0.002;
  config.q_axis_inductance_h = 0.001;
  config.pole_pairs = 2;

  mca::plant::PmsmDq motor(config);
  ASSERT_TRUE(motor.reset(0.0, 0.0, 4.0, 3.0).ok());
  // 1.5*2*(0.05*3 + (0.002-0.001)*4*3) = 0.486 Nm.
  EXPECT_NEAR(motor.electromagnetic_torque(), 0.486, 1e-12);
}

TEST(PmsmDq, MechanicalDynamicsUseElectromagneticTorque) {
  mca::plant::PmsmDqConfig config;
  config.permanent_magnet_flux_wb = 0.1;
  config.inertia_kg_m2 = 0.01;
  config.viscous_damping_nm_per_rad_s = 0.1;
  config.pole_pairs = 2;

  mca::plant::PmsmDq motor(config);
  ASSERT_TRUE(motor.reset(0.0, 0.0, 0.0, 1.0).ok());
  ASSERT_TRUE(motor.step(0.0, 0.0, 0.0, 0.001).ok());

  // Te=1.5*2*0.1*1=0.3 Nm; alpha=30 rad/s^2; w=0.03 rad/s.
  EXPECT_NEAR(motor.velocity(), 0.03, 1e-12);
  EXPECT_NEAR(motor.position(), 0.00003, 1e-12);
}

TEST(PmsmDq, ElectricalAngleUsesPolePairs) {
  mca::plant::PmsmDqConfig config;
  config.pole_pairs = 7;
  mca::plant::PmsmDq motor(config);
  ASSERT_TRUE(motor.reset(0.2, 1.5, 0.0, 0.0).ok());
  EXPECT_NEAR(motor.electrical_angle(), 1.4, 1e-12);
  EXPECT_NEAR(motor.electrical_angular_velocity(), 10.5, 1e-12);
}

}  // namespace
