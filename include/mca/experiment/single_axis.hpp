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
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    config_ = experiment_config;
    plant_ = plant;
    if (!plant_.configured()) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    if (!pid_.configure(pid_config).ok() || !ff_.configure(ff_config).ok()) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    step_metrics_enabled_ = std::abs(config_.reference) >
                            std::numeric_limits<double>::epsilon();
    if (step_metrics_enabled_ &&
        !step_metrics_.configure(0.0, config_.reference, 0.02, 0.05)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    configured_ = true;
    return reset();
  }

  core::Status reset() noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    accumulator_.reset();
    step_metrics_.reset();
    plant_.reset();
    pid_.reset(0.0, 0.0);
    return {};
  }

  core::Status run(SingleAxisResult& result) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    accumulator_.reset();
    step_metrics_.reset();
    plant_.reset();
    pid_.reset(0.0, plant_.position());
    const trajectory::StepTrajectory reference_trajectory(config_.reference);

    for (int i = 0; i < config_.steps; ++i) {
      const double time_s = static_cast<double>(i) * config_.dt;
      const auto reference = reference_trajectory.sample(time_s);
      double feedback = 0.0;
      const auto status = pid_.update(reference.position, plant_.position(),
                                      config_.dt, feedback);
      if (!status.ok()) return status;

      const double effort = feedback + ff_.compute(reference.velocity,
                                                   reference.acceleration);
      const auto plant_status = plant_.step(effort, config_.dt);
      if (!plant_status.ok()) return plant_status;
      const double sample_time_s = time_s + config_.dt;
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
