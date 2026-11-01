#pragma once

#include "mca/plant/bldc_pmsm.hpp"

namespace mca::plant::vendor_profiles {

// Official maxon EC-i 40, 48 V, Hall-sensor variant, part number 496653.
// Values below are transcribed from the manufacturer's product page/catalog.
// Terminal R/L are phase-to-phase measurements and must not be silently
// treated as the per-phase parameters of BldcPmsm's uncoupled phase model.
struct MaxonEcI40_496653 {
  static constexpr const char* manufacturer = "maxon";
  static constexpr const char* model = "EC-i 40";
  static constexpr const char* part_number = "496653";

  static constexpr double nominal_voltage_v = 48.0;
  static constexpr double no_load_speed_rpm = 7560.0;
  static constexpr double nominal_speed_rpm = 6310.0;
  static constexpr double nominal_torque_nm = 0.0739;
  static constexpr double continuous_current_a = 1.19;
  static constexpr double stall_torque_nm = 0.526;
  static constexpr double stall_current_a = 18.5;
  static constexpr double terminal_resistance_phase_to_phase_ohm = 2.59;
  static constexpr double terminal_inductance_phase_to_phase_h = 1.93e-3;
  static constexpr double torque_constant_nm_per_a = 0.0596;
  static constexpr double speed_constant_rpm_per_v = 160.0;
  static constexpr double rotor_inertia_kg_m2 = 1.28e-6;  // 12.8 g cm^2
  static constexpr double max_speed_rpm = 10000.0;
  static constexpr unsigned pole_pairs = 7;
  static constexpr unsigned phases = 3;

  // Electrical mapping is deliberately caller-supplied. The datasheet gives
  // terminal phase-to-phase R/L; the simple BldcPmsm model expects uncoupled
  // per-phase R/L. Winding connection and mutual inductance must be resolved
  // before selecting these values for a physically faithful model.
  struct ModelMapping {
    double phase_resistance_ohm{0.0};
    double phase_inductance_h{0.0};
    double viscous_damping_nm_per_rad_s{0.0};
    ThreePhaseMotorKind kind{ThreePhaseMotorKind::PmsmSinusoidal};
  };

  [[nodiscard]] static BldcPmsmConfig make_model_config(
      const ModelMapping& mapping) noexcept {
    BldcPmsmConfig config;
    config.kind = mapping.kind;
    config.phase_resistance_ohm = mapping.phase_resistance_ohm;
    config.phase_inductance_h = mapping.phase_inductance_h;
    config.torque_constant_nm_per_a = torque_constant_nm_per_a;
    // Convert rpm/V to V/(rad/s). This is a nominal catalog conversion;
    // the exact phase/line convention must match the chosen motor model.
    config.back_emf_constant_v_per_rad_s =
        60.0 / (6.28318530717958647692 * speed_constant_rpm_per_v);
    config.inertia_kg_m2 = rotor_inertia_kg_m2;
    config.viscous_damping_nm_per_rad_s =
        mapping.viscous_damping_nm_per_rad_s;
    config.pole_pairs = pole_pairs;
    return config;
  }
};

}  // namespace mca::plant::vendor_profiles
