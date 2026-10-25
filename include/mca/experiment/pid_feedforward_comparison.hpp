#pragma once

#include "mca/benchmark/metrics.hpp"
#include "mca/control/feedforward.hpp"
#include "mca/control/pid.hpp"
#include "mca/core/types.hpp"
#include "mca/plant/second_order.hpp"
#include "mca/trajectory/scalar.hpp"

namespace mca::experiment {

struct PidFeedforwardComparisonConfig {
  double dt{0.001};
  int steps{2000};
  double initial_position{0.0};
  double ramp_target{1.0};
  double ramp_duration_s{1.0};
  double settling_tolerance_ratio{0.02};
  double settling_hold_time_s{0.05};
  control::PidConfig pid{};
  control::PidFeedforwardConfig feedforward{};
  plant::SecondOrderPlant plant{1.0, 0.5, 0.0};
};

struct PidFeedforwardCaseResult {
  benchmark::TrackingMetrics tracking{};
  double final_position{0.0};
  double final_velocity{0.0};
  double peak_position{0.0};
};

struct PidFeedforwardComparisonResult {
  PidFeedforwardCaseResult pid_only{};
  PidFeedforwardCaseResult pid_plus_feedforward{};
  double rmse_reduction_fraction{0.0};
  double control_effort_reduction_fraction{0.0};
};

// Deterministic comparison of the same PID controller with and without
// model-based velocity/acceleration feedforward on a second-order plant.
// Both cases use identical initial state, reference trajectory and timestep.
// The runner owns no dynamic memory and has no ROS2/Gazebo dependency.
class PidFeedforwardComparison {
 public:
  core::Status configure(const PidFeedforwardComparisonConfig& config) noexcept {
    if (!(config.dt > 0.0) || config.steps <= 0 ||
        !core::is_finite(config.dt) || !core::is_finite(config.initial_position) ||
        !core::is_finite(config.ramp_target) || !(config.ramp_duration_s > 0.0) ||
        !core::is_finite(config.ramp_duration_s) ||
        !(config.settling_tolerance_ratio > 0.0) ||
        !(config.settling_tolerance_ratio < 1.0) ||
        !core::is_finite(config.settling_tolerance_ratio) ||
        config.settling_hold_time_s < 0.0 ||
        !core::is_finite(config.settling_hold_time_s) ||
        config.pid.output_min > config.pid.output_max ||
        config.pid.integral_min > config.pid.integral_max ||
        !core::is_finite(config.pid.kp) || !core::is_finite(config.pid.ki) ||
        !core::is_finite(config.pid.kd) ||
        !core::is_finite(config.pid.output_min) || !core::is_finite(config.pid.output_max) ||
        !core::is_finite(config.pid.integral_min) || !core::is_finite(config.pid.integral_max) ||
        !core::is_finite(config.feedforward.velocity_gain) ||
        !core::is_finite(config.feedforward.acceleration_gain) ||
        !core::is_finite(config.feedforward.constant_bias)) {
      return {core::StatusCode::InvalidArgument};
    }

    // Build a complete candidate so rejected reconfiguration leaves the
    // previously working comparison and its runtime state untouched.
    auto trial = *this;
    if (!trial.pid_only_.configure(config.pid).ok() ||
        !trial.pid_ff_.configure(config.pid).ok() ||
        !trial.plant_only_.configure(config.plant.inertia(), config.plant.damping(),
                                    config.plant.stiffness()).ok() ||
        !trial.plant_ff_.configure(config.plant.inertia(), config.plant.damping(),
                                   config.plant.stiffness()).ok()) {
      return {core::StatusCode::InvalidArgument};
    }
    trial.config_ = config;
    trial.feedforward_ = control::PidFeedforward(config.feedforward);
    trial.configured_ = true;
    if (auto status = trial.reset_impl(); !status.ok()) return status;
    *this = trial;
    return {};
  }

  core::Status reset() noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    auto trial = *this;
    if (auto status = trial.reset_impl(); !status.ok()) return status;
    *this = trial;
    return {};
  }

  core::Status run(PidFeedforwardComparisonResult& result) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    auto trial = *this;
    PidFeedforwardComparisonResult trial_result{};
    const auto status = trial.run_impl(trial_result);
    if (!status.ok()) return status;
    *this = trial;
    result = trial_result;
    return {};
  }

 private:
  core::Status reset_impl() noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!pid_only_.reset().ok() || !pid_ff_.reset().ok() ||
        !plant_only_.reset(config_.initial_position, 0.0).ok() ||
        !plant_ff_.reset(config_.initial_position, 0.0).ok()) {
      return {core::StatusCode::InvalidArgument};
    }
    return {};
  }

  core::Status run_impl(PidFeedforwardComparisonResult& result) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (auto status = reset_impl(); !status.ok()) return status;

    trajectory::RampTrajectory reference(config_.initial_position,
                                         config_.ramp_target,
                                         config_.ramp_duration_s);
    benchmark::TrackingAccumulator pid_metrics;
    benchmark::TrackingAccumulator ff_metrics;
    benchmark::StepResponseAccumulator pid_step;
    benchmark::StepResponseAccumulator ff_step;
    if (!pid_step.configure(config_.initial_position, config_.ramp_target,
                            config_.settling_tolerance_ratio,
                            config_.settling_hold_time_s) ||
        !ff_step.configure(config_.initial_position, config_.ramp_target,
                           config_.settling_tolerance_ratio,
                           config_.settling_hold_time_s)) {
      return {core::StatusCode::InvalidArgument};
    }

    for (int i = 0; i < config_.steps; ++i) {
      const double time_s = static_cast<double>(i) * config_.dt;
      const auto sample = reference.sample(time_s);
      double pid_output = 0.0;
      double ff_pid_output = 0.0;
      if (!pid_only_.update(sample.position, plant_only_.position(), config_.dt,
                            pid_output).ok() ||
          !pid_ff_.update(sample.position, plant_ff_.position(), config_.dt,
                          ff_pid_output).ok()) {
        return {core::StatusCode::InvalidArgument};
      }
      const double combined_output =
          ff_pid_output + feedforward_.compute(sample.velocity, sample.acceleration);
      if (!core::is_finite(combined_output)) {
        return {core::StatusCode::NumericalError};
      }
      if (!plant_only_.step(pid_output, config_.dt).ok() ||
          !plant_ff_.step(combined_output, config_.dt).ok()) {
        return {core::StatusCode::NumericalError};
      }

      const double sample_time_s = time_s + config_.dt;
      if (!core::is_finite(time_s) || !core::is_finite(sample_time_s)) {
        return {core::StatusCode::NumericalError};
      }
      pid_metrics.add(sample.position, plant_only_.position(), pid_output);
      ff_metrics.add(sample.position, plant_ff_.position(), combined_output);
      pid_step.add(sample_time_s, plant_only_.position());
      ff_step.add(sample_time_s, plant_ff_.position());
    }

    result.pid_only.tracking = pid_metrics.result();
    result.pid_only.tracking.overshoot = pid_step.overshoot();
    result.pid_only.tracking.settling_time_s = pid_step.settling_time_s();
    result.pid_only.final_position = plant_only_.position();
    result.pid_only.final_velocity = plant_only_.velocity();
    result.pid_only.peak_position =
        config_.ramp_target + result.pid_only.tracking.overshoot *
                                  (config_.ramp_target - config_.initial_position);

    result.pid_plus_feedforward.tracking = ff_metrics.result();
    result.pid_plus_feedforward.tracking.overshoot = ff_step.overshoot();
    result.pid_plus_feedforward.tracking.settling_time_s = ff_step.settling_time_s();
    result.pid_plus_feedforward.final_position = plant_ff_.position();
    result.pid_plus_feedforward.final_velocity = plant_ff_.velocity();
    result.pid_plus_feedforward.peak_position =
        config_.ramp_target + result.pid_plus_feedforward.tracking.overshoot *
                                  (config_.ramp_target - config_.initial_position);

    const double baseline_rmse = result.pid_only.tracking.rmse;
    const double baseline_effort = result.pid_only.tracking.control_effort_rms;
    result.rmse_reduction_fraction =
        baseline_rmse > 0.0
            ? (baseline_rmse - result.pid_plus_feedforward.tracking.rmse) / baseline_rmse
            : 0.0;
    result.control_effort_reduction_fraction =
        baseline_effort > 0.0
            ? (baseline_effort - result.pid_plus_feedforward.tracking.control_effort_rms) /
                  baseline_effort
            : 0.0;
    if (!core::is_finite(result.pid_only.peak_position) ||
        !core::is_finite(result.pid_plus_feedforward.peak_position) ||
        !core::is_finite(result.rmse_reduction_fraction) ||
        !core::is_finite(result.control_effort_reduction_fraction) ||
        !core::is_finite(result.pid_only.final_position) ||
        !core::is_finite(result.pid_only.final_velocity) ||
        !core::is_finite(result.pid_plus_feedforward.final_position) ||
        !core::is_finite(result.pid_plus_feedforward.final_velocity)) {
      return {core::StatusCode::NumericalError};
    }
    return {};
  }

 public:
  [[nodiscard]] bool configured() const noexcept { return configured_; }

 private:
  PidFeedforwardComparisonConfig config_{};
  control::PidController pid_only_{};
  control::PidController pid_ff_{};
  control::PidFeedforward feedforward_{};
  plant::SecondOrderPlant plant_only_{};
  plant::SecondOrderPlant plant_ff_{};
  bool configured_{false};
};

}  // namespace mca::experiment
