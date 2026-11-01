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
              60.0 / (6.28318530717958647692 * 160.0), 1e-15);

  mca::plant::BldcPmsm motor;
  EXPECT_TRUE(motor.configure(config).ok());
}
