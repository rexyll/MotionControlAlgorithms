#pragma once

#include "mca/benchmark/metrics.hpp"
#include "mca/control/feedforward.hpp"
#include "mca/control/pid.hpp"
#include "mca/plant/second_order.hpp"

namespace mca::experiment {

struct SingleAxisConfig {
  double dt{0.001};
  int steps{10000};
  double reference{1.0};
};

struct SingleAxisResult {
  benchmark::BenchmarkResult metrics{};
  double final_position{0.0};
  double final_velocity{0.0};
};

// Minimal deterministic single-axis closed-loop experiment. The experiment
// composes a PID controller, optional model-based feedforward, and a second-
// order mechanical plant without ROS2/Gazebo dependencies.
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
    configured_ = true;
    return reset();
  }

  core::Status reset() noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    accumulator_.reset();
    plant_.reset();
    pid_.reset(0.0, 0.0);
    return {};
  }

  core::Status run(SingleAxisResult& result) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    accumulator_.reset();
    plant_.reset();
    pid_.reset(0.0, plant_.position());

    for (int i = 0; i < config_.steps; ++i) {
      double feedback = 0.0;
      const auto status = pid_.update(config_.reference, plant_.position(),
                                      config_.dt, feedback);
      if (!status.ok()) return status;

      // For this first experiment the reference is a step: v_ref=a_ref=0.
      const double effort = feedback + ff_.compute(0.0, 0.0);
      if (!plant_.step(effort, config_.dt).ok()) {
        return {core::StatusCode::RuntimeError};
      }
      accumulator_.add(config_.reference, plant_.position(), effort);
    }

    result.metrics = accumulator_.result();
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
  bool configured_{false};
};

}  // namespace mca::experiment
