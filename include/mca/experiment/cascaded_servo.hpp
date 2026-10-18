#pragma once

#include <cmath>
#include <limits>

#include "mca/benchmark/metrics.hpp"
#include "mca/control/cascaded_servo.hpp"
#include "mca/core/math.hpp"
#include "mca/core/types.hpp"
#include "mca/plant/dc_motor.hpp"
#include "mca/trajectory/scalar.hpp"

namespace mca::experiment {

struct CascadedServoConfig {
  double dt{0.0001};
  int steps{10000};
  double position_reference{1.0};
  double load_torque_nm{0.0};
  double max_voltage_v{12.0};
  double max_current_a{10.0};
  double settling_tolerance_ratio{0.02};
  double settling_hold_time_s{0.05};
};

struct CascadedServoResult {
  benchmark::TrackingMetrics position{};
  benchmark::TrackingMetrics velocity{};
  benchmark::TrackingMetrics torque{};
  benchmark::ConstraintMetrics constraints{};
  double final_position{0.0};
  double final_velocity{0.0};
  double final_current{0.0};
  double final_torque{0.0};
  double final_voltage{0.0};
};

// Deterministic scalar position -> velocity -> torque benchmark around a DC
// motor. The cascade remains independent from the plant; the experiment is
// only the composition used to validate tracking and actuator constraints.
class CascadedServoExperiment {
 public:
  core::Status configure(const CascadedServoConfig& experiment_config,
                         const control::CascadedServoConfig& servo_config,
                         const plant::DcMotorConfig& motor_config) noexcept {
    if (!(experiment_config.dt > 0.0) || experiment_config.steps <= 0 ||
        !core::is_finite(experiment_config.dt) ||
        !core::is_finite(experiment_config.position_reference) ||
        !core::is_finite(experiment_config.load_torque_nm) ||
        !(experiment_config.max_voltage_v > 0.0) ||
        !(experiment_config.max_current_a > 0.0) ||
        !core::is_finite(experiment_config.max_voltage_v) ||
        !core::is_finite(experiment_config.max_current_a) ||
        !(experiment_config.settling_tolerance_ratio > 0.0) ||
        !(experiment_config.settling_hold_time_s >= 0.0) ||
        !core::is_finite(experiment_config.settling_tolerance_ratio) ||
        !core::is_finite(experiment_config.settling_hold_time_s)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    if (!servo_.configure(servo_config).ok()) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    if (!motor_.configure(motor_config).ok()) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    if (!position_step_.configure(0.0, experiment_config.position_reference,
                                  experiment_config.settling_tolerance_ratio,
                                  experiment_config.settling_hold_time_s)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    config_ = experiment_config;
    configured_ = true;
    return reset();
  }

  core::Status reset() noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    motor_.reset();
    servo_.reset();
    position_accumulator_.reset();
    velocity_accumulator_.reset();
    torque_accumulator_.reset();
    position_step_.reset();
    constraints_ = {};
    final_voltage_ = 0.0;
    return {};
  }

  core::Status run(CascadedServoResult& result) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!reset().ok()) return {core::StatusCode::InvalidArgument};

    const trajectory::StepTrajectory reference(config_.position_reference);
    for (int i = 0; i < config_.steps; ++i) {
      const double time_s = static_cast<double>(i) * config_.dt;
      const auto sample = reference.sample(time_s);
      double voltage = 0.0;
      if (!servo_.update(sample.position, motor_.position(), motor_.velocity(),
                         motor_.torque(), config_.dt, voltage)
               .ok()) {
        return {core::StatusCode::InvalidArgument};
      }

      if (std::abs(voltage) > config_.max_voltage_v) {
        const double violation = std::abs(voltage) - config_.max_voltage_v;
        constraints_.violations++;
        constraints_.max_violation = math::max(constraints_.max_violation, violation);
        voltage = math::clamp(voltage, -config_.max_voltage_v, config_.max_voltage_v);
      }
      if (std::abs(motor_.current()) > config_.max_current_a) {
        const double violation = std::abs(motor_.current()) - config_.max_current_a;
        constraints_.violations++;
        constraints_.max_violation = math::max(constraints_.max_violation, violation);
      }

      if (!motor_.step(voltage, config_.load_torque_nm, config_.dt).ok()) {
        return {core::StatusCode::InvalidArgument};
      }

      const double sample_time_s = time_s + config_.dt;
      position_accumulator_.add(sample.position, motor_.position(), voltage);
      velocity_accumulator_.add(servo_.velocity_reference(), motor_.velocity(),
                                voltage);
      torque_accumulator_.add(servo_.torque_reference(), motor_.torque(), voltage);
      position_step_.add(sample_time_s, motor_.position());
      final_voltage_ = voltage;
    }

    result.position = position_accumulator_.result();
    result.velocity = velocity_accumulator_.result();
    result.torque = torque_accumulator_.result();
    result.position.overshoot = position_step_.overshoot();
    result.position.settling_time_s = position_step_.settling_time_s();
    result.constraints = constraints_;
    result.final_position = motor_.position();
    result.final_velocity = motor_.velocity();
    result.final_current = motor_.current();
    result.final_torque = motor_.torque();
    result.final_voltage = final_voltage_;
    return {};
  }

  [[nodiscard]] const control::CascadedServo& controller() const noexcept {
    return servo_;
  }
  [[nodiscard]] const plant::DcMotor& motor() const noexcept { return motor_; }

 private:
  CascadedServoConfig config_{};
  control::CascadedServo servo_{};
  plant::DcMotor motor_{};
  benchmark::TrackingAccumulator position_accumulator_{};
  benchmark::TrackingAccumulator velocity_accumulator_{};
  benchmark::TrackingAccumulator torque_accumulator_{};
  benchmark::StepResponseAccumulator position_step_{};
  benchmark::ConstraintMetrics constraints_{};
  double final_voltage_{0.0};
  bool configured_{false};
};

}  // namespace mca::experiment
