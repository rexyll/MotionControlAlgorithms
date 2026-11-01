#include <cmath>

#include <gtest/gtest.h>

#include "mca/plant/vendor_profiles/maxon_ec_i_40_496653.hpp"

TEST(MaxonEcI40Profile, PreservesOfficialCatalogValuesInSiUnits) {
  using Profile = mca::plant::vendor_profiles::MaxonEcI40_496653;
  EXPECT_STREQ(Profile::manufacturer, "maxon");
  EXPECT_STREQ(Profile::model, "EC-i 40");
  EXPECT_STREQ(Profile::part_number, "496653");
  EXPECT_DOUBLE_EQ(Profile::nominal_voltage_v, 48.0);
  EXPECT_DOUBLE_EQ(Profile::nominal_speed_rpm, 6310.0);
  EXPECT_DOUBLE_EQ(Profile::nominal_torque_nm, 0.0739);
  EXPECT_DOUBLE_EQ(Profile::continuous_current_a, 1.19);
  EXPECT_DOUBLE_EQ(Profile::terminal_resistance_phase_to_phase_ohm, 2.59);
  EXPECT_DOUBLE_EQ(Profile::terminal_inductance_phase_to_phase_h, 0.00193);
  EXPECT_DOUBLE_EQ(Profile::torque_constant_nm_per_a, 0.0596);
  EXPECT_DOUBLE_EQ(Profile::rotor_inertia_kg_m2, 1.28e-6);
  EXPECT_EQ(Profile::pole_pairs, 7U);
}

TEST(MaxonEcI40Profile, RequiresExplicitPhaseElectricalMappingForPlantModel) {
  using Profile = mca::plant::vendor_profiles::MaxonEcI40_496653;
  // Illustrative star/no-mutual-inductance approximation only. The values
  // are caller assumptions, not a claim about the motor winding topology.
  Profile::ModelMapping mapping;
  mapping.phase_resistance_ohm =
      Profile::terminal_resistance_phase_to_phase_ohm / 2.0;
  mapping.phase_inductance_h =
      Profile::terminal_inductance_phase_to_phase_h / 2.0;
  mapping.viscous_damping_nm_per_rad_s = 0.0001;  // simulation assumption

  const auto config = Profile::make_model_config(mapping);
  EXPECT_DOUBLE_EQ(config.phase_resistance_ohm, 1.295);
  EXPECT_DOUBLE_EQ(config.phase_inductance_h, 0.000965);
  EXPECT_DOUBLE_EQ(config.torque_constant_nm_per_a, 0.0596);
  EXPECT_DOUBLE_EQ(config.inertia_kg_m2, 1.28e-6);
  EXPECT_EQ(config.pole_pairs, 7U);
  EXPECT_NEAR(config.back_emf_constant_v_per_rad_s,
              60.0 /
                  (6.28318530717958647692 * 160.0 * 7.0),
              1e-15);

  mca::plant::BldcPmsm motor;
  EXPECT_TRUE(motor.configure(config).ok());
}

TEST(MaxonEcI40Profile, CatalogSpeedConstantMapsToNoLoadVoltage) {
  using Profile = mca::plant::vendor_profiles::MaxonEcI40_496653;
  Profile::ModelMapping mapping;
  mapping.phase_resistance_ohm =
      Profile::terminal_resistance_phase_to_phase_ohm / 2.0;
  mapping.phase_inductance_h =
      Profile::terminal_inductance_phase_to_phase_h / 2.0;
  mapping.viscous_damping_nm_per_rad_s = 0.0001;

  mca::plant::BldcPmsm motor(Profile::make_model_config(mapping));
  const double no_load_speed_rad_s =
      Profile::no_load_speed_rpm * 2.0 * 3.14159265358979323846 / 60.0;
  ASSERT_TRUE(motor.reset(0.0, no_load_speed_rad_s).ok());

  // BldcPmsm reports the phase back-EMF coefficient applied to electrical
  // speed. At the catalog mechanical no-load speed, the resulting phase
  // back-EMF magnitude should equal the catalog speed-constant voltage.
  const double expected_voltage =
      Profile::no_load_speed_rpm / Profile::speed_constant_rpm_per_v;
  const double electrical_speed =
      no_load_speed_rad_s * static_cast<double>(Profile::pole_pairs);
  const double modeled_voltage =
      motor.config().back_emf_constant_v_per_rad_s * electrical_speed;
  EXPECT_NEAR(modeled_voltage, expected_voltage, 1e-12);
  EXPECT_NEAR(expected_voltage, 47.25, 1e-12);
}

TEST(MaxonEcI40Profile, DynamicSynchronousExcitationHasCatalogConsistentSpeedScale) {
  using Profile = mca::plant::vendor_profiles::MaxonEcI40_496653;
  Profile::ModelMapping mapping;
  mapping.phase_resistance_ohm =
      Profile::terminal_resistance_phase_to_phase_ohm / 2.0;
  mapping.phase_inductance_h =
      Profile::terminal_inductance_phase_to_phase_h / 2.0;
  mapping.viscous_damping_nm_per_rad_s = 0.0001;  // simulation assumption

  mca::plant::BldcPmsm motor(Profile::make_model_config(mapping));
  constexpr double kPi = 3.14159265358979323846;
  constexpr double dt = 2.0e-6;
  constexpr double phase_voltage_peak_v = 10.0;
  constexpr int steps = 10000;  // 20 ms

  ASSERT_TRUE(motor.reset().ok());
  double max_abs_current = 0.0;
  for (int i = 0; i < steps; ++i) {
    const double theta = motor.electrical_angle();
    const double va = phase_voltage_peak_v * std::sin(theta);
    const double vb =
        phase_voltage_peak_v * std::sin(theta - 2.0 * kPi / 3.0);
    const double vc =
        phase_voltage_peak_v * std::sin(theta + 2.0 * kPi / 3.0);
    ASSERT_TRUE(motor.step(va, vb, vc, 0.0, dt).ok());
    max_abs_current =
        std::max({max_abs_current, std::abs(motor.current_a()),
                  std::abs(motor.current_b()), std::abs(motor.current_c())});
  }

  const double speed_rpm = motor.speed() * 60.0 / (2.0 * kPi);
  EXPECT_TRUE(std::isfinite(speed_rpm));
  EXPECT_GT(speed_rpm, 1400.0);
  EXPECT_LT(speed_rpm, 1700.0);
  EXPECT_LT(max_abs_current, 4.0);

  // This is a software plant simulation only. The damping and phase R/L
  // mapping are explicit approximations; the result is not HIL or physical
  // motor validation.
}
