#pragma once

#include "mca/core/math.hpp"
#include "mca/core/types.hpp"

namespace mca::control {

struct SlidingModeConfig {
  double switching_gain{1.0};
  double boundary_layer{0.02};
  double output_min{-1.0e9};
  double output_max{1.0e9};
};

struct SlidingModeOutput {
  double nominal_control{0.0};
  double switching_control{0.0};
  double control{0.0};
  bool saturated{false};
};

// Scalar boundary-layer sliding-mode law: u = u_eq - k*sat(s/phi).
// The caller supplies the sliding variable and equivalent/nominal control.
// The boundary layer replaces sign(s) with a continuous approximation.
class SlidingModeController {
 public:
  SlidingModeController() = default;
  explicit SlidingModeController(const SlidingModeConfig& config) { configure(config); }

  core::Status configure(const SlidingModeConfig& config) noexcept {
    if (!valid(config)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    config_ = config;
    configured_ = true;
    return {};
  }

  core::Status update(double sliding_variable, double nominal_control,
                      SlidingModeOutput& output) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(sliding_variable) || !core::is_finite(nominal_control))
      return {core::StatusCode::InvalidArgument};

    const double normalized = math::clamp(
        sliding_variable / config_.boundary_layer, -1.0, 1.0);
    const double switching = -config_.switching_gain * normalized;
    const double unsaturated = nominal_control + switching;
    const double control = math::clamp(
        unsaturated, config_.output_min, config_.output_max);

    SlidingModeOutput candidate;
    candidate.nominal_control = nominal_control;
    candidate.switching_control = switching;
    candidate.control = control;
    candidate.saturated = control != unsaturated;
    output = candidate;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] const SlidingModeConfig& config() const noexcept { return config_; }

 private:
  static bool valid(const SlidingModeConfig& c) noexcept {
    return core::is_finite(c.switching_gain) && c.switching_gain >= 0.0 &&
           core::is_finite(c.boundary_layer) && c.boundary_layer > 0.0 &&
           core::is_finite(c.output_min) && core::is_finite(c.output_max) &&
           c.output_min <= c.output_max;
  }

  SlidingModeConfig config_{};
  bool configured_{false};
};

}  // namespace mca::control
