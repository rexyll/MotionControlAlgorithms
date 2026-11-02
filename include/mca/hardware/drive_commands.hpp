#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "mca/core/types.hpp"
#include "mca/hardware/device_metadata.hpp"

namespace mca::hardware {

struct DriveSetpoint {
  std::size_t axis{0};
  DriveControlMode control_mode{DriveControlMode::Position};
  DriveOperationMode operation_mode{DriveOperationMode::ProfilePosition};
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double torque_nm{0.0};
  double current_a{0.0};

  [[nodiscard]] bool valid() const noexcept {
    switch (control_mode) {
      case DriveControlMode::Position:
        return normalized_control_mode(operation_mode) == control_mode &&
               std::isfinite(position_rad);
      case DriveControlMode::Velocity:
        return normalized_control_mode(operation_mode) == control_mode &&
               std::isfinite(velocity_rad_s);
      case DriveControlMode::Torque:
        return normalized_control_mode(operation_mode) == control_mode &&
               std::isfinite(torque_nm);
      case DriveControlMode::Current:
        // Direct current control is a vendor-specific capability, not a
        // standardized CiA 402 PDS operation mode.
        return std::isfinite(current_a);
    }
    return false;
  }
};

template <std::size_t MaxAxes>
struct DriveCommandBatch {
  static_assert(MaxAxes > 0, "DriveCommandBatch requires positive capacity");
  std::array<DriveSetpoint, MaxAxes> setpoints{};
  std::size_t count{0};
  HardwareCommandMetadata metadata{};

  [[nodiscard]] bool valid_shape() const noexcept {
    if (count == 0U || count > MaxAxes || !metadata.valid()) return false;
    for (std::size_t i = 0; i < count; ++i) {
      if (!setpoints[i].valid()) return false;
      for (std::size_t j = 0; j < i; ++j) {
        if (setpoints[i].axis == setpoints[j].axis) return false;
      }
    }
    return true;
  }
};

// Performs the complete validation pass before a backend is allowed to touch
// transport state. last_accepted_sequence is owned by the sink and advances
// only after the whole batch has been accepted.
template <std::size_t MaxAxes>
core::Status validate_drive_command_batch(
    const DeviceCapabilities<MaxAxes>& capabilities,
    const DriveCommandBatch<MaxAxes>& batch,
    std::uint64_t now_ns, HardwareClockDomain now_domain,
    std::uint64_t last_accepted_sequence) noexcept {
  if (!capabilities.valid() || !batch.valid_shape()) {
    return {core::StatusCode::InvalidArgument};
  }
  if (batch.metadata.sequence <= last_accepted_sequence) {
    return {core::StatusCode::InvalidArgument};
  }
  if (!batch.metadata.deadline_compatible(now_domain) ||
      batch.metadata.expired(now_ns, now_domain)) {
    return {core::StatusCode::RuntimeError};
  }
  for (std::size_t i = 0; i < batch.count; ++i) {
    const auto& setpoint = batch.setpoints[i];
    if (setpoint.axis >= capabilities.axis_count) {
      return {core::StatusCode::ConstraintViolation};
    }
    const auto& axis = capabilities.axes[setpoint.axis];
    if (setpoint.control_mode == DriveControlMode::Current) {
      if (!axis.supports(DriveControlMode::Current)) {
        return {core::StatusCode::ConstraintViolation};
      }
    } else if (!axis.supports(setpoint.operation_mode) ||
               !axis.supports(setpoint.control_mode)) {
      return {core::StatusCode::ConstraintViolation};
    }
  }
  return {};
}

// Implementations must validate the entire batch before I/O and must document
// transport-level atomicity. A backend that cannot atomically apply a multi-axis
// command must reject count > 1 unless it has a safe staged/commit mechanism.
template <std::size_t MaxAxes>
class IDriveCommandSink {
 public:
  virtual ~IDriveCommandSink() = default;
  virtual core::Status write_batch(
      const DriveCommandBatch<MaxAxes>& batch,
      std::uint64_t now_ns, HardwareClockDomain now_domain) noexcept = 0;
  [[nodiscard]] virtual std::uint64_t last_accepted_sequence() const noexcept = 0;
};

}  // namespace mca::hardware
