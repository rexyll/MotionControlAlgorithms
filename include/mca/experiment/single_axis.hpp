#pragma once

#include <cmath>
#include <limits>

#include "mca/benchmark/metrics.hpp"
#include "mca/control/feedforward.hpp"
#include "mca/control/pid.hpp"
#include "mca/plant/second_order.hpp"
#include "mca/trajectory/scalar.hpp"

namespace mca::experiment {

struct SingleAxisConfig {
  double dt{0.001};
  int steps{10000};
  double reference{1.0};
};

struct SingleAxisResult {
  benchmark::TrackingMetrics metrics{};
  double final_position{0.0};
  double final_velocity{0.0};
};

class SingleAxisExperiment {
 public:
  core::Status configure(const SingleAxisConfig& experiment_config,
                         const plant::SecondOrderPlant& plant,
                         const control::PidConfig& pid_config,
                         const control::PidFeedforwardConfig& ff_config = {}) noexcept {
    if (!(experiment_config.dt > 0.0) || experiment_config.steps <= 0 ||
        !core::is_finite(experiment_config.dt) ||
        !core::is_finite(experiment_config.reference)) {
      return {core::StatusCode::InvalidArgument};
    }

    // The final sample time is steps * dt. Finite inputs can still produce
    // an unrepresentable simulation horizon, so reject it before mutation.
    const double final_sample_time =
        static_cast<double>(experiment_config.steps) * experiment_config.dt;
    if (!core::is_finite(final_sample_time)) {
      return {core::StatusCode::NumericalError};
    }

    // Configure a complete trial object first. A rejected experiment update
    // must not disable or partially replace an already usable experiment.
    auto trial = *this;
    trial.config_ = experiment_config;
    trial.plant_ = plant;
    if (!trial.plant_.configured()) {
      return {core::StatusCode::InvalidArgument};
    }
    if (auto status = trial.pid_.configure(pid_config); !status.ok()) {
      return status;
    }
    if (auto status = trial.ff_.configure(ff_config); !status.ok()) {
      return status;
    }

    trial.step_metrics_enabled_ =
        std::abs(trial.config_.reference) >
        std::numeric_limits<double>::epsilon();
    if (trial.step_metrics_enabled_ &&
        !trial.step_metrics_.configure(0.0, trial.config_.reference, 0.02,
                                       0.05)) {
      return {core::StatusCode::InvalidArgument};
    }

    trial.configured_ = true;
    if (auto status = trial.reset_impl(); !status.ok()) return status;
    *this = trial;
    return {};
  }

  core::Status reset() noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    // Reset all runtime state on a candidate so a partial component reset
    // cannot leave the active experiment internally inconsistent.
    auto trial = *this;
    if (auto status = trial.reset_impl(); !status.ok()) return status;
    *this = trial;
    return {};
  }

  core::Status run(SingleAxisResult& result) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};

    // Run the full experiment on a trial copy so a controller/plant failure
    // after several successful samples cannot expose partially advanced state
    // or overwrite the caller's result.
    auto trial = *this;
    SingleAxisResult trial_result{};
    const auto status = trial.run_impl(trial_result);
    if (!status.ok()) return status;
    *this = trial;
    result = trial_result;
    return {};
  }

 private:
  static bool tracking_metrics_valid(const benchmark::TrackingMetrics& metrics,
                                     int expected_samples) noexcept {
    const bool settling_is_nan = metrics.settling_time_s != metrics.settling_time_s;
    return metrics.samples == static_cast<std::size_t>(expected_samples) &&
           core::is_finite(metrics.rmse) &&
           core::is_finite(metrics.max_abs_error) &&
           core::is_finite(metrics.overshoot) &&
           core::is_finite(metrics.control_effort_rms) &&
           (core::is_finite(metrics.settling_time_s) || settling_is_nan);
  }

  core::Status reset_impl() noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    accumulator_.reset();
    step_metrics_.reset();
    if (auto status = plant_.reset(); !status.ok()) return status;
    if (auto status = pid_.reset(0.0, 0.0); !status.ok()) return status;
    return {};
  }

  core::Status run_impl(SingleAxisResult& result) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (auto status = reset_impl(); !status.ok()) return status;

    const trajectory::StepTrajectory reference_trajectory(config_.reference);
    for (int i = 0; i < config_.steps; ++i) {
      const double time_s = static_cast<double>(i) * config_.dt;
      if (!core::is_finite(time_s)) {
        return {core::StatusCode::NumericalError};
      }
      const auto reference = reference_trajectory.sample(time_s);
      double feedback = 0.0;
      const auto status = pid_.update(reference.position, plant_.position(),
                                      config_.dt, feedback);
      if (!status.ok()) return status;

      const double effort = feedback +
                            ff_.compute(reference.velocity,
                                        reference.acceleration);
      if (!core::is_finite(effort)) {
        return {core::StatusCode::NumericalError};
      }
      const auto plant_status = plant_.step(effort, config_.dt);
      if (!plant_status.ok()) return plant_status;
      const double sample_time_s = time_s + config_.dt;
      if (!core::is_finite(sample_time_s)) {
        return {core::StatusCode::NumericalError};
      }
      accumulator_.add(reference.position, plant_.position(), effort);
      if (step_metrics_enabled_) {
        step_metrics_.add(sample_time_s, plant_.position());
      }
    }

    result.metrics = accumulator_.result();
    if (step_metrics_enabled_) {
      result.metrics.overshoot = step_metrics_.overshoot();
      result.metrics.settling_time_s = step_metrics_.settling_time_s();
    }
    result.final_position = plant_.position();
    result.final_velocity = plant_.velocity();
    if (!tracking_metrics_valid(result.metrics, config_.steps) ||
        !core::is_finite(result.final_position) ||
        !core::is_finite(result.final_velocity)) {
      return {core::StatusCode::NumericalError};
    }
    return {};
  }

 private:
  SingleAxisConfig config_{};
  plant::SecondOrderPlant plant_{};
  control::PidController pid_{};
  control::PidFeedforward ff_{};
  benchmark::TrackingAccumulator accumulator_{};
  benchmark::StepResponseAccumulator step_metrics_{};
  bool step_metrics_enabled_{false};
  bool configured_{false};
};

}  // namespace mca::experiment