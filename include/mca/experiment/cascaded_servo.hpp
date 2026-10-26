#pragma once

#include <cmath>

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
        !(experiment_config.settling_tolerance_ratio < 1.0) ||
        !(experiment_config.settling_hold_time_s >= 0.0) ||
        !core::is_finite(experiment_config.settling_tolerance_ratio) ||
        !core::is_finite(experiment_config.settling_hold_time_s)) {
      return {core::StatusCode::InvalidArgument};
    }

    // The loop records the final sample at steps * dt. Reject a finite
    // timestep/sample-count pair whose derived simulation horizon is not
    // representable before replacing an already-working experiment.
    const double final_sample_time =
        static_cast<double>(experiment_config.steps) * experiment_config.dt;
    if (!core::is_finite(final_sample_time)) {
      return {core::StatusCode::NumericalError};
    }

    // Configure copies first. A failure in any component must not disable or
    // partially replace a previously usable experiment.
    auto servo_trial = servo_;
    auto motor_trial = motor_;
    auto position_step_trial = position_step_;
    if (auto status = servo_trial.configure(servo_config); !status.ok()) {
      return status;
    }
    if (auto status = motor_trial.configure(motor_config); !status.ok()) {
      return status;
    }
    if (!position_step_trial.configure(
            0.0, experiment_config.position_reference,
            experiment_config.settling_tolerance_ratio,
            experiment_config.settling_hold_time_s)) {
      return {core::StatusCode::InvalidArgument};
    }
    if (auto status = servo_trial.reset(); !status.ok()) return status;
    if (auto status = motor_trial.reset(); !status.ok()) return status;

    config_ = experiment_config;
    servo_ = servo_trial;
    motor_ = motor_trial;
    position_step_ = position_step_trial;
    position_accumulator_.reset();
    velocity_accumulator_.reset();
    torque_accumulator_.reset();
    constraints_ = {};
    final_voltage_ = 0.0;
    configured_ = true;
    return {};
  }

  core::Status reset() noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};

    // Reset every component on a candidate so a partial failure cannot leave
    // the active experiment with mismatched plant/controller/metric state.
    auto trial = *this;
    if (auto status = trial.reset_impl(); !status.ok()) return status;
    *this = trial;
    return {};
  }

  core::Status run(CascadedServoResult& result) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};

    // Execute the full benchmark on a trial copy. A controller/plant failure
    // may occur after several successful steps; do not expose a partially
    // advanced experiment or overwrite the caller's result in that case.
    auto trial = *this;
    CascadedServoResult trial_result{};
    const auto status = trial.run_impl(trial_result);
    if (!status.ok()) return status;
    *this = trial;
    result = trial_result;
    return {};
  }

 private:
  core::Status reset_impl() noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (auto status = motor_.reset(); !status.ok()) return status;
    if (auto status = servo_.reset(); !status.ok()) return status;
    position_accumulator_.reset();
    velocity_accumulator_.reset();
    torque_accumulator_.reset();
    position_step_.reset();
    constraints_ = {};
    final_voltage_ = 0.0;
    return {};
  }

  static bool tracking_metrics_valid(const benchmark::TrackingMetrics& metrics,
                                     int expected_samples) noexcept {
    const bool settling_is_nan =
        metrics.settling_time_s != metrics.settling_time_s;
    return metrics.samples == static_cast<std::size_t>(expected_samples) &&
           core::is_finite(metrics.rmse) &&
           core::is_finite(metrics.max_abs_error) &&
           core::is_finite(metrics.overshoot) &&
           core::is_finite(metrics.control_effort_rms) &&
           (core::is_finite(metrics.settling_time_s) || settling_is_nan);
  }

  core::Status run_impl(CascadedServoResult& result) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};

    const double final_sample_time =
        static_cast<double>(config_.steps) * config_.dt;
    if (!core::is_finite(final_sample_time)) {
      return {core::StatusCode::NumericalError};
    }

    if (auto status = reset(); !status.ok()) return status;

    const trajectory::StepTrajectory reference(config_.position_reference);
    for (int i = 0; i < config_.steps; ++i) {
      const double time_s = static_cast<double>(i) * config_.dt;
      if (!core::is_finite(time_s)) {
        return {core::StatusCode::NumericalError};
      }
      const auto sample = reference.sample(time_s);
      double voltage = 0.0;
      const auto control_status = servo_.update(
          sample.position, motor_.position(), motor_.velocity(), motor_.torque(),
          config_.dt, voltage);
      if (!control_status.ok()) return control_status;

      if (!core::is_finite(voltage)) {
        return {core::StatusCode::NumericalError};
      }
      if (std::abs(voltage) > config_.max_voltage_v) {
        const double violation = std::abs(voltage) - config_.max_voltage_v;
        if (!core::is_finite(violation)) {
          return {core::StatusCode::NumericalError};
        }
        ++constraints_.violations;
        constraints_.max_violation =
            std::max(constraints_.max_violation, violation);
        if (!core::is_finite(constraints_.max_violation)) {
          return {core::StatusCode::NumericalError};
        }
        voltage = math::clamp(voltage, -config_.max_voltage_v,
                              config_.max_voltage_v);
      }
      const auto plant_status =
          motor_.step(voltage, config_.load_torque_nm, config_.dt);
      if (!plant_status.ok()) return plant_status;

      // Current is a plant state updated by step(). Check the post-step value
      // so a threshold crossing on the final sample is not silently missed.
      if (!core::is_finite(motor_.current())) {
        return {core::StatusCode::NumericalError};
      }
      if (std::abs(motor_.current()) > config_.max_current_a) {
        const double violation =
            std::abs(motor_.current()) - config_.max_current_a;
        if (!core::is_finite(violation)) {
          return {core::StatusCode::NumericalError};
        }
        ++constraints_.violations;
        constraints_.max_violation =
            std::max(constraints_.max_violation, violation);
        if (!core::is_finite(constraints_.max_violation)) {
          return {core::StatusCode::NumericalError};
        }
      }

      const double sample_time_s = time_s + config_.dt;
      if (!core::is_finite(sample_time_s)) {
        return {core::StatusCode::NumericalError};
      }
      position_accumulator_.add(sample.position, motor_.position(), voltage);
      velocity_accumulator_.add(servo_.velocity_reference(), motor_.velocity(),
                                voltage);
      torque_accumulator_.add(servo_.torque_reference(), motor_.torque(),
                              voltage);
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

    if (!tracking_metrics_valid(result.position, config_.steps) ||
        !tracking_metrics_valid(result.velocity, config_.steps) ||
        !tracking_metrics_valid(result.torque, config_.steps) ||
        !core::is_finite(result.final_position) ||
        !core::is_finite(result.final_velocity) ||
        !core::is_finite(result.final_current) ||
        !core::is_finite(result.final_torque) ||
        !core::is_finite(result.final_voltage) ||
        !core::is_finite(result.constraints.max_violation)) {
      return {core::StatusCode::NumericalError};
    }
    return {};
  }

 public:
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
