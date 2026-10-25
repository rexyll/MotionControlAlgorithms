#include <gtest/gtest.h>
#include <cmath>

#include <limits>

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

TEST(PmsmDq, InvalidReconfigurationPreservesWorkingState) {
  mca::plant::PmsmDq motor(mca::plant::PmsmDqConfig{});
  ASSERT_TRUE(motor.reset(0.25, 0.5, 0.1, -0.2).ok());
  const auto before_config = motor.config();
  mca::plant::PmsmDqConfig invalid = before_config;
  invalid.pole_pairs = 0;

  EXPECT_EQ(motor.configure(invalid).code, mca::core::StatusCode::InvalidArgument);
  EXPECT_TRUE(motor.configured());
  EXPECT_EQ(motor.config().pole_pairs, before_config.pole_pairs);
  EXPECT_DOUBLE_EQ(motor.position(), 0.25);
  EXPECT_DOUBLE_EQ(motor.velocity(), 0.5);
  EXPECT_DOUBLE_EQ(motor.d_current(), 0.1);
  EXPECT_DOUBLE_EQ(motor.q_current(), -0.2);
  EXPECT_TRUE(motor.step(0.0, 0.0, 0.0, 0.001).ok());
}

TEST(PmsmDq, ResetDerivedOverflowPreservesState) {
  mca::plant::PmsmDq motor(mca::plant::PmsmDqConfig{});
  ASSERT_TRUE(motor.reset(0.25, 0.5, 0.1, -0.2).ok());

  const auto status = motor.reset(std::numeric_limits<double>::max(), 0.5,
                                  0.1, -0.2);
  EXPECT_EQ(status.code, mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(motor.position(), 0.25);
  EXPECT_DOUBLE_EQ(motor.velocity(), 0.5);
  EXPECT_DOUBLE_EQ(motor.d_current(), 0.1);
  EXPECT_DOUBLE_EQ(motor.q_current(), -0.2);
  EXPECT_TRUE(motor.step(0.0, 0.0, 0.0, 0.001).ok());
}

TEST(PmsmDq, NumericalOverflowDoesNotCommitState) {
  mca::plant::PmsmDqConfig config;
  config.d_axis_inductance_h = 1e-308;
  mca::plant::PmsmDq motor(config);
  ASSERT_TRUE(motor.reset(0.25, 0.0, 0.0, 0.0).ok());

  const auto status = motor.step(std::numeric_limits<double>::max(), 0.0,
                                 0.0, 1.0);
  EXPECT_EQ(status.code, mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(motor.position(), 0.25);
  EXPECT_DOUBLE_EQ(motor.velocity(), 0.0);
  EXPECT_DOUBLE_EQ(motor.d_current(), 0.0);
  EXPECT_DOUBLE_EQ(motor.q_current(), 0.0);
  EXPECT_TRUE(motor.step(0.0, 0.0, 0.0, 0.001).ok());
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

  // we=20 rad/s. Explicit Euler evaluates both coupled derivatives from
  // the same pre-step state:
  // did/dt=(-2 + 20*0.003*3)/0.002 = -910 A/s
  // diq/dt=(-3 - 20*(0.002*2+0.1))/0.003 = -1693.333... A/s
  EXPECT_NEAR(motor.d_current(), 1.909, 1e-12);
  EXPECT_NEAR(motor.q_current(), 2.830666666666667, 1e-12);
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

  // The current itself decays during the step because Rq/Lq=500 1/s:
  // iq=1-0.5/0.001*0.001=0.5 A, hence Te=0.15 Nm. The resulting
  // acceleration is 15 rad/s^2 and semi-implicit position is w*dt.
  EXPECT_NEAR(motor.velocity(), 0.015, 1e-12);
  EXPECT_NEAR(motor.position(), 0.000015, 1e-12);
}

TEST(PmsmDq, ElectricalAngleUsesPolePairs) {
  mca::plant::PmsmDqConfig config;
  config.pole_pairs = 7;
  mca::plant::PmsmDq motor(config);
  ASSERT_TRUE(motor.reset(0.2, 1.5, 0.0, 0.0).ok());
  EXPECT_NEAR(motor.electrical_angle(), 1.4, 1e-12);
  EXPECT_NEAR(motor.electrical_angular_velocity(), 10.5, 1e-12);
}

TEST(PmsmDq, CheckedElectricalStateReportsStatusAndMatchesLegacy) {
  mca::plant::PmsmDqConfig config;
  config.pole_pairs = 7;
  mca::plant::PmsmDq motor(config);
  ASSERT_TRUE(motor.reset(0.2, 1.5, 0.0, 0.0).ok());

  double angle = -1.0;
  double speed = -2.0;
  EXPECT_TRUE(motor.electrical_angle(angle).ok());
  EXPECT_TRUE(motor.electrical_angular_velocity(speed).ok());
  EXPECT_NEAR(angle, motor.electrical_angle(), 1e-12);
  EXPECT_NEAR(speed, motor.electrical_angular_velocity(), 1e-12);
}

TEST(PmsmDq, CheckedElectricalStateReportsNotConfiguredAndPreservesOutput) {
  mca::plant::PmsmDq motor;
  double angle = 11.0;
  double speed = 12.0;

  EXPECT_EQ(motor.electrical_angle(angle).code,
            mca::core::StatusCode::NotConfigured);
  EXPECT_EQ(motor.electrical_angular_velocity(speed).code,
            mca::core::StatusCode::NotConfigured);
  EXPECT_DOUBLE_EQ(angle, 11.0);
  EXPECT_DOUBLE_EQ(speed, 12.0);
}

TEST(PmsmDq, CheckedElectricalStateRejectsOverflowAndPreservesOutput) {
  mca::plant::PmsmDqConfig config;
  config.pole_pairs = 2;
  mca::plant::PmsmDq motor(config);
  ASSERT_TRUE(motor.reset(0.25, 0.5, 0.0, 0.0).ok());

  double angle = 21.0;
  double speed = 22.0;
  ASSERT_EQ(motor.reset(std::numeric_limits<double>::max(), 0.5, 0.0, 0.0).code,
            mca::core::StatusCode::NumericalError);
  ASSERT_EQ(motor.reset(0.25, std::numeric_limits<double>::max(), 0.0, 0.0).code,
            mca::core::StatusCode::NumericalError);

  // Failed resets preserve the previous finite state; diagnostics must return
  // that valid state instead of expecting an unreachable invalid runtime value.
  EXPECT_TRUE(motor.electrical_angle(angle).ok());
  EXPECT_TRUE(motor.electrical_angular_velocity(speed).ok());
  EXPECT_NEAR(angle, 0.5, 1e-12);
  EXPECT_NEAR(speed, 1.0, 1e-12);
}

TEST(PmsmDq, CheckedAccelerationReportsStatusAndPreservesOutput) {
  mca::plant::PmsmDq unconfigured;
  double output = 123.0;
  EXPECT_EQ(unconfigured.acceleration(0.0, output).code,
            mca::core::StatusCode::NotConfigured);
  EXPECT_DOUBLE_EQ(output, 123.0);

  mca::plant::PmsmDq motor;
  ASSERT_TRUE(motor.configure(mca::plant::PmsmDqConfig{}).ok());
  ASSERT_TRUE(motor.reset(0.0, 2.0, 1.0, 3.0).ok());
  EXPECT_EQ(motor.acceleration(std::numeric_limits<double>::infinity(), output).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_DOUBLE_EQ(output, 123.0);

  double torque = 0.0;
  ASSERT_TRUE(motor.electromagnetic_torque(torque).ok());
  const double expected = (torque - motor.config().viscous_damping_nm_per_rad_s *
                           motor.velocity() - 0.2) / motor.config().inertia_kg_m2;
  ASSERT_TRUE(motor.acceleration(0.2, output).ok());
  EXPECT_DOUBLE_EQ(output, expected);
  EXPECT_DOUBLE_EQ(motor.velocity(), 2.0);
  EXPECT_DOUBLE_EQ(motor.q_current(), 3.0);
}

TEST(PmsmDq, CheckedAccelerationRejectsOverflowWithoutChangingOutput) {
  mca::plant::PmsmDqConfig config;
  config.viscous_damping_nm_per_rad_s = std::numeric_limits<double>::max() / 2.0;
  mca::plant::PmsmDq motor;
  ASSERT_TRUE(motor.configure(config).ok());
  ASSERT_TRUE(motor.reset(0.0, 4.0, 0.0, 0.0).ok());
  double output = -77.0;
  EXPECT_EQ(motor.acceleration(0.0, output).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(output, -77.0);
  EXPECT_DOUBLE_EQ(motor.velocity(), 4.0);
}

TEST(PmsmDq, CheckedElectromagneticTorqueReportsStatusAndMatchesLegacy) {
  mca::plant::PmsmDqConfig config;
  config.permanent_magnet_flux_wb = 0.05;
  config.d_axis_inductance_h = 0.002;
  config.q_axis_inductance_h = 0.001;
  config.pole_pairs = 2;

  mca::plant::PmsmDq motor(config);
  ASSERT_TRUE(motor.reset(0.0, 0.0, 4.0, 3.0).ok());

  double checked = -123.0;
  const auto status = motor.electromagnetic_torque(checked);
  EXPECT_TRUE(status.ok());
  EXPECT_NEAR(checked, 0.486, 1e-12);
  EXPECT_DOUBLE_EQ(checked, motor.electromagnetic_torque());
}

TEST(PmsmDq, CheckedElectromagneticTorqueReportsNotConfiguredAndPreservesOutput) {
  mca::plant::PmsmDq motor;
  double checked = 7.5;

  const auto status = motor.electromagnetic_torque(checked);
  EXPECT_EQ(status.code, mca::core::StatusCode::NotConfigured);
  EXPECT_DOUBLE_EQ(checked, 7.5);
}

TEST(PmsmDq, CheckedElectromagneticTorqueUsesFiniteLifecycleState) {
  mca::plant::PmsmDqConfig config;
  config.d_axis_inductance_h = 0.002;
  config.q_axis_inductance_h = 0.001;
  mca::plant::PmsmDq motor(config);
  ASSERT_TRUE(motor.reset(0.0, 0.0, 0.0, 0.0).ok());
  double checked = 9.0;

  // With nonzero saliency, DBL_MAX d/q currents make the derived torque
  // overflow. Public reset must reject that candidate before it can enter
  // runtime state; the checked accessor must still expose the prior valid
  // state and leave the caller output unchanged on the failed reset.
  const auto reset_status = motor.reset(0.0, 0.0,
                                        std::numeric_limits<double>::max(),
                                        std::numeric_limits<double>::max());
  EXPECT_EQ(reset_status.code, mca::core::StatusCode::NumericalError);
  const auto torque_status = motor.electromagnetic_torque(checked);
  EXPECT_TRUE(torque_status.ok());
  EXPECT_DOUBLE_EQ(checked, 0.0);
}


TEST(PmsmDq, CheckedMechanicalPowerReportsStatusAndPreservesState) {
  mca::plant::PmsmDq motor;
  double output = 123.0;
  EXPECT_EQ(motor.mechanical_power(output).code,
            mca::core::StatusCode::NotConfigured);
  EXPECT_DOUBLE_EQ(output, 123.0);

  mca::plant::PmsmDqConfig config;
  config.permanent_magnet_flux_wb = 0.05;
  config.pole_pairs = 2;
  mca::plant::PmsmDq configured(config);
  ASSERT_TRUE(configured.reset(0.25, 3.0, 0.0, 4.0).ok());

  double torque = 0.0;
  ASSERT_TRUE(configured.electromagnetic_torque(torque).ok());
  const double expected = torque * configured.velocity();
  const double before_position = configured.position();
  const double before_velocity = configured.velocity();
  const double before_d = configured.d_current();
  const double before_q = configured.q_current();

  ASSERT_TRUE(configured.mechanical_power(output).ok());
  EXPECT_DOUBLE_EQ(output, expected);
  EXPECT_DOUBLE_EQ(configured.position(), before_position);
  EXPECT_DOUBLE_EQ(configured.velocity(), before_velocity);
  EXPECT_DOUBLE_EQ(configured.d_current(), before_d);
  EXPECT_DOUBLE_EQ(configured.q_current(), before_q);
}

TEST(PmsmDq, CheckedMechanicalPowerRejectsOverflowWithoutChangingOutputOrState) {
  mca::plant::PmsmDqConfig config;
  config.permanent_magnet_flux_wb =
      std::numeric_limits<double>::max() / 10.0;
  config.pole_pairs = 4;
  mca::plant::PmsmDq motor(config);
  ASSERT_TRUE(motor.reset(0.0, 2.0, 0.0, 1.0).ok());

  double torque = 0.0;
  ASSERT_TRUE(motor.electromagnetic_torque(torque).ok());
  ASSERT_TRUE(std::isfinite(torque));
  ASSERT_GT(std::abs(torque), std::numeric_limits<double>::max() / 2.0);

  const double before_position = motor.position();
  const double before_velocity = motor.velocity();
  const double before_d = motor.d_current();
  const double before_q = motor.q_current();
  double output = -77.0;
  EXPECT_EQ(motor.mechanical_power(output).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(output, -77.0);
  EXPECT_DOUBLE_EQ(motor.position(), before_position);
  EXPECT_DOUBLE_EQ(motor.velocity(), before_velocity);
  EXPECT_DOUBLE_EQ(motor.d_current(), before_d);
  EXPECT_DOUBLE_EQ(motor.q_current(), before_q);
}


TEST(PmsmDq, CheckedElectricalPowerReportsStatusAndPreservesState) {
  mca::plant::PmsmDq unconfigured;
  double output = 123.0;
  EXPECT_EQ(unconfigured.electrical_power(4.0, 5.0, output).code,
            mca::core::StatusCode::NotConfigured);
  EXPECT_DOUBLE_EQ(output, 123.0);

  mca::plant::PmsmDq motor;
  ASSERT_TRUE(motor.configure({}).ok());
  ASSERT_TRUE(motor.reset(0.2, 1.5, 2.0, -3.0).ok());
  EXPECT_EQ(motor.electrical_power(std::numeric_limits<double>::infinity(),
                                   5.0, output).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_DOUBLE_EQ(output, 123.0);
  ASSERT_TRUE(motor.electrical_power(4.0, 5.0, output).ok());
  EXPECT_DOUBLE_EQ(output, -10.5);
  EXPECT_DOUBLE_EQ(motor.position(), 0.2);
  EXPECT_DOUBLE_EQ(motor.velocity(), 1.5);
  EXPECT_DOUBLE_EQ(motor.d_current(), 2.0);
  EXPECT_DOUBLE_EQ(motor.q_current(), -3.0);
}

TEST(PmsmDq, CheckedElectricalPowerRejectsOverflowWithoutChangingOutput) {
  mca::plant::PmsmDqConfig config;
  config.permanent_magnet_flux_wb = 0.01;
  config.pole_pairs = 2;
  mca::plant::PmsmDq motor(config);
  ASSERT_TRUE(motor.reset(0.0, 0.0, 0.0,
                          std::numeric_limits<double>::max()).ok());
  double output = -77.0;
  EXPECT_EQ(motor.electrical_power(0.0, std::numeric_limits<double>::max(),
                                   output).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(output, -77.0);
  EXPECT_DOUBLE_EQ(motor.q_current(), std::numeric_limits<double>::max());
}

TEST(PmsmDq, CheckedCopperLossReportsStatusAndPreservesState) {
  mca::plant::PmsmDq unconfigured;
  double output = 123.0;
  EXPECT_EQ(unconfigured.copper_loss(output).code,
            mca::core::StatusCode::NotConfigured);
  EXPECT_DOUBLE_EQ(output, 123.0);

  mca::plant::PmsmDq motor;
  ASSERT_TRUE(motor.configure({}).ok());
  ASSERT_TRUE(motor.reset(0.2, 1.5, 2.0, -3.0).ok());
  ASSERT_TRUE(motor.copper_loss(output).ok());
  EXPECT_DOUBLE_EQ(output, 1.5 * 0.5 * (4.0 + 9.0));
  EXPECT_DOUBLE_EQ(motor.position(), 0.2);
  EXPECT_DOUBLE_EQ(motor.velocity(), 1.5);
  EXPECT_DOUBLE_EQ(motor.d_current(), 2.0);
  EXPECT_DOUBLE_EQ(motor.q_current(), -3.0);
}

TEST(PmsmDq, CheckedCopperLossRejectsOverflowWithoutChangingOutput) {
  mca::plant::PmsmDq motor;
  ASSERT_TRUE(motor.configure({}).ok());
  ASSERT_TRUE(motor.reset(0.0, 0.0, 0.0,
                          std::numeric_limits<double>::max()).ok());
  double output = -77.0;
  EXPECT_EQ(motor.copper_loss(output).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(output, -77.0);
  EXPECT_DOUBLE_EQ(motor.q_current(), std::numeric_limits<double>::max());
}


TEST(PmsmDq, PowerDiagnosticsHandleZeroAndNegativeDqCurrentConvention) {
  mca::plant::PmsmDq motor;
  ASSERT_TRUE(motor.configure({}).ok());
  ASSERT_TRUE(motor.reset(0.0, 0.0, -2.0, 0.0).ok());
  double electrical = 99.0;
  double copper = 99.0;
  ASSERT_TRUE(motor.electrical_power(3.0, 0.0, electrical).ok());
  ASSERT_TRUE(motor.copper_loss(copper).ok());
  EXPECT_DOUBLE_EQ(electrical, -9.0);
  EXPECT_DOUBLE_EQ(copper, 3.0);
  ASSERT_TRUE(motor.reset(0.0, 0.0, 0.0, 0.0).ok());
  ASSERT_TRUE(motor.electrical_power(0.0, 0.0, electrical).ok());
  ASSERT_TRUE(motor.copper_loss(copper).ok());
  EXPECT_DOUBLE_EQ(electrical, 0.0);
  EXPECT_DOUBLE_EQ(copper, 0.0);
}

}  // namespace

TEST(PmsmDq, CheckedCopperLossRejectsOverflowingCurrentSquareSum) {
  mca::plant::PmsmDqConfig config;
  config.d_axis_inductance_h = 0.001;
  config.q_axis_inductance_h = 0.001;
  config.permanent_magnet_flux_wb = 1e-300;
  mca::plant::PmsmDq motor(config);
  const double current = std::sqrt(std::numeric_limits<double>::max()) * 0.9;
  ASSERT_TRUE(motor.reset(0.25, 1.5, current, current).ok());

  const double before_position = motor.position();
  const double before_velocity = motor.velocity();
  const double before_d = motor.d_current();
  const double before_q = motor.q_current();
  double output = 123.0;

  // Each current square is finite, but their sum overflows.
  EXPECT_EQ(motor.copper_loss(output).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(output, 123.0);
  EXPECT_DOUBLE_EQ(motor.position(), before_position);
  EXPECT_DOUBLE_EQ(motor.velocity(), before_velocity);
  EXPECT_DOUBLE_EQ(motor.d_current(), before_d);
  EXPECT_DOUBLE_EQ(motor.q_current(), before_q);
}

TEST(PmsmDq, CheckedElectricalPowerRejectsOverflowingSumWithoutChangingOutputOrState) {
  mca::plant::PmsmDqConfig config;
  // Equal inductances and zero PM flux make the diagnostic operating point
  // torque-neutral even with very large finite currents.
  config.d_axis_inductance_h = 0.001;
  config.q_axis_inductance_h = 0.001;
  config.permanent_magnet_flux_wb = 1e-300;
  mca::plant::PmsmDq motor(config);
  const double max = std::numeric_limits<double>::max();
  ASSERT_TRUE(motor.reset(0.25, 1.5, max, max).ok());

  const double before_position = motor.position();
  const double before_velocity = motor.velocity();
  const double before_d = motor.d_current();
  const double before_q = motor.q_current();
  double output = 123.0;

  // Each product is finite; their sum overflows before dq scaling.
  EXPECT_EQ(motor.electrical_power(1.0, 1.0, output).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(output, 123.0);
  EXPECT_DOUBLE_EQ(motor.position(), before_position);
  EXPECT_DOUBLE_EQ(motor.velocity(), before_velocity);
  EXPECT_DOUBLE_EQ(motor.d_current(), before_d);
  EXPECT_DOUBLE_EQ(motor.q_current(), before_q);
}

TEST(PmsmDq, CheckedElectricalPowerRejectsNonFiniteVoltageOnBothAxes) {
  mca::plant::PmsmDq motor;
  ASSERT_TRUE(motor.configure({}).ok());
  ASSERT_TRUE(motor.reset(0.25, 1.5, 1.0, -2.0).ok());

  double output = 123.0;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_EQ(motor.electrical_power(nan, 0.0, output).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_DOUBLE_EQ(output, 123.0);
  EXPECT_EQ(motor.electrical_power(0.0, inf, output).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_DOUBLE_EQ(output, 123.0);

  EXPECT_DOUBLE_EQ(motor.position(), 0.25);
  EXPECT_DOUBLE_EQ(motor.velocity(), 1.5);
  EXPECT_DOUBLE_EQ(motor.d_current(), 1.0);
  EXPECT_DOUBLE_EQ(motor.q_current(), -2.0);
}
