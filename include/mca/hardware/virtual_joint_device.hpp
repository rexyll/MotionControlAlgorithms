#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "mca/hardware/interfaces.hpp"

namespace mca::hardware {

// Per-axis parameters for a deliberately simple virtual rotary joint. These
// defaults are simulation assumptions, not manufacturer specifications.
template <std::size_t Dimensions>
struct VirtualJointDeviceConfig {
  static_assert(Dimensions > 0, "A virtual device needs at least one joint");

  std::array<double, Dimensions> inertia_kg_m2{};
  std::array<double, Dimensions> viscous_damping_nm_s_rad{};
  std::array<double, Dimensions> stiffness_nm_rad{};
  std::array<double, Dimensions> effort_limit_nm{};
  std::array<double, Dimensions> position_min_rad{};
  std::array<double, Dimensions> position_max_rad{};
  std::array<double, Dimensions> velocity_limit_rad_s{};
  std::array<double, Dimensions> initial_position_rad{};
  double maximum_step_seconds{0.01};

  VirtualJointDeviceConfig() noexcept {
    inertia_kg_m2.fill(0.02);
    viscous_damping_nm_s_rad.fill(0.02);
    stiffness_nm_rad.fill(0.0);
    effort_limit_nm.fill(5.0);
    position_min_rad.fill(-3.14159265358979323846);
    position_max_rad.fill(3.14159265358979323846);
    velocity_limit_rad_s.fill(10.0);
    initial_position_rad.fill(0.0);
  }

  [[nodiscard]] bool valid() const noexcept {
    if (!std::isfinite(maximum_step_seconds) || maximum_step_seconds <= 0.0) {
      return false;
    }
    for (std::size_t i = 0; i < Dimensions; ++i) {
      if (!std::isfinite(inertia_kg_m2[i]) || inertia_kg_m2[i] <= 0.0 ||
          !std::isfinite(viscous_damping_nm_s_rad[i]) ||
          viscous_damping_nm_s_rad[i] < 0.0 ||
          !std::isfinite(stiffness_nm_rad[i]) || stiffness_nm_rad[i] < 0.0 ||
          !std::isfinite(effort_limit_nm[i]) || effort_limit_nm[i] <= 0.0 ||
          !std::isfinite(position_min_rad[i]) ||
          !std::isfinite(position_max_rad[i]) ||
          position_min_rad[i] >= position_max_rad[i] ||
          !std::isfinite(velocity_limit_rad_s[i]) ||
          velocity_limit_rad_s[i] <= 0.0 ||
          !std::isfinite(initial_position_rad[i]) ||
          initial_position_rad[i] < position_min_rad[i] ||
          initial_position_rad[i] > position_max_rad[i]) {
        return false;
      }
    }
    return true;
  }
};

// Deterministic N-axis virtual drive/plant. It implements the same lifecycle,
// coherent-state and effort-command ports used by physical backends. It is
// intentionally single-thread-owned; a real adapter must serialize access or
// provide its own bounded, real-time-safe synchronization.
template <std::size_t Dimensions>
class VirtualJointDevice final : public IHardwareLifecycle,
                                 public IJointStateSource<double, Dimensions>,
                                 public IJointEffortCommandSink<double, Dimensions> {
 public:
  using State = JointState<double, Dimensions>;
  using Command = JointEffortCommand<double, Dimensions>;
  using Config = VirtualJointDeviceConfig<Dimensions>;

  explicit VirtualJointDevice(const Config& config = Config{}) noexcept
      : config_(config) {}

  core::Status initialize() noexcept override {
    if (lifecycle_ != HardwareLifecycleState::Uninitialized || !config_.valid()) {
      return {core::StatusCode::InvalidArgument};
    }
    position_ = config_.initial_position_rad;
    velocity_.fill(0.0);
    acceleration_.fill(0.0);
    applied_effort_.fill(0.0);
    sample_sequence_ = 0;
    lifecycle_ = HardwareLifecycleState::Inactive;
    return {};
  }

  core::Status activate() noexcept override {
    if (lifecycle_ != HardwareLifecycleState::Inactive) {
      return {core::StatusCode::NotConfigured};
    }
    lifecycle_ = HardwareLifecycleState::Active;
    return {};
  }

  core::Status deactivate() noexcept override {
    if (lifecycle_ != HardwareLifecycleState::Active) {
      return {core::StatusCode::NotConfigured};
    }
    applied_effort_.fill(0.0);
    lifecycle_ = HardwareLifecycleState::Inactive;
    return {};
  }

  [[nodiscard]] HardwareLifecycleState lifecycle_state() const noexcept override {
    return lifecycle_;
  }

  core::Status read_state(State& output) noexcept override {
    if (lifecycle_ != HardwareLifecycleState::Inactive &&
        lifecycle_ != HardwareLifecycleState::Active) {
      return {core::StatusCode::NotConfigured};
    }
    if (sample_sequence_ == UINT64_MAX) {
      return {core::StatusCode::RuntimeError};
    }
    State sample;
    sample.position = position_;
    sample.velocity = velocity_;
    sample.acceleration = acceleration_;
    sample.effort = applied_effort_;
    sample.sequence = sample_sequence_ + 1U;
    sample.valid_fields = State::kPositionValid | State::kVelocityValid |
                          State::kAccelerationValid | State::kEffortValid;
    if (!sample.valid()) return {core::StatusCode::NumericalError};
    sample_sequence_ = sample.sequence;
    output = sample;
    return {};
  }

  core::Status write_effort(const Command& command) noexcept override {
    if (lifecycle_ != HardwareLifecycleState::Active) {
      return {core::StatusCode::NotConfigured};
    }
    if (!command.valid()) return {core::StatusCode::InvalidArgument};
    for (std::size_t i = 0; i < Dimensions; ++i) {
      if (std::abs(command.effort[i]) > config_.effort_limit_nm[i]) {
        return {core::StatusCode::ConstraintViolation};
      }
    }
    applied_effort_ = command.effort;
    return {};
  }

  core::Status request_safe_stop() noexcept override {
    if (lifecycle_ != HardwareLifecycleState::Active) {
      return {core::StatusCode::NotConfigured};
    }
    applied_effort_.fill(0.0);
    return {};
  }

  // Advance the virtual plant by one bounded integration step. This is called
  // by the simulator/test harness, not by the generic hardware port.
  core::Status step(double dt_seconds) noexcept {
    if (lifecycle_ != HardwareLifecycleState::Active) {
      return {core::StatusCode::NotConfigured};
    }
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0 ||
        dt_seconds > config_.maximum_step_seconds) {
      return {core::StatusCode::InvalidArgument};
    }

    auto next_position = position_;
    auto next_velocity = velocity_;
    auto next_acceleration = acceleration_;
    for (std::size_t i = 0; i < Dimensions; ++i) {
      const double restoring = config_.stiffness_nm_rad[i] * position_[i];
      const double damping = config_.viscous_damping_nm_s_rad[i] * velocity_[i];
      const double acceleration =
          (applied_effort_[i] - damping - restoring) / config_.inertia_kg_m2[i];
      double velocity = velocity_[i] + acceleration * dt_seconds;
      if (velocity > config_.velocity_limit_rad_s[i]) {
        velocity = config_.velocity_limit_rad_s[i];
      } else if (velocity < -config_.velocity_limit_rad_s[i]) {
        velocity = -config_.velocity_limit_rad_s[i];
      }
      double position = position_[i] + velocity * dt_seconds;
      if (position > config_.position_max_rad[i]) {
        position = config_.position_max_rad[i];
        if (velocity > 0.0) velocity = 0.0;
      } else if (position < config_.position_min_rad[i]) {
        position = config_.position_min_rad[i];
        if (velocity < 0.0) velocity = 0.0;
      }
      const double actual_acceleration = (velocity - velocity_[i]) / dt_seconds;
      if (!std::isfinite(position) || !std::isfinite(velocity) ||
          !std::isfinite(actual_acceleration)) {
        return {core::StatusCode::NumericalError};
      }
      next_position[i] = position;
      next_velocity[i] = velocity;
      next_acceleration[i] = actual_acceleration;
    }
    position_ = next_position;
    velocity_ = next_velocity;
    acceleration_ = next_acceleration;
    return {};
  }

  [[nodiscard]] const Config& config() const noexcept { return config_; }

 private:
  Config config_{};
  HardwareLifecycleState lifecycle_{HardwareLifecycleState::Uninitialized};
  std::array<double, Dimensions> position_{};
  std::array<double, Dimensions> velocity_{};
  std::array<double, Dimensions> acceleration_{};
  std::array<double, Dimensions> applied_effort_{};
  std::uint64_t sample_sequence_{0};
};

}  // namespace mca::hardware
