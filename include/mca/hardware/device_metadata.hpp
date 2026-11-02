#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace mca::hardware {

enum class DriveControlMode : std::uint8_t {
  Position = 1U << 0U,
  Velocity = 1U << 1U,
  Torque = 1U << 2U,
  Current = 1U << 3U
};

constexpr std::uint8_t control_mode_bit(DriveControlMode mode) noexcept {
  return static_cast<std::uint8_t>(mode);
}

constexpr std::uint8_t kKnownDriveControlModes =
    control_mode_bit(DriveControlMode::Position) |
    control_mode_bit(DriveControlMode::Velocity) |
    control_mode_bit(DriveControlMode::Torque) |
    control_mode_bit(DriveControlMode::Current);

// CiA 402 operation modes are deliberately separate from normalized MCA
// command semantics: e.g. Profile Position and CSP are both position modes,
// but their trajectory/timing contracts are not interchangeable.
enum class DriveOperationMode : std::uint16_t {
  ProfilePosition = 1U << 0U,
  ProfileVelocity = 1U << 1U,
  ProfileTorque = 1U << 2U,
  CyclicSynchronousPosition = 1U << 3U,
  CyclicSynchronousVelocity = 1U << 4U,
  CyclicSynchronousTorque = 1U << 5U,
  Homing = 1U << 6U
};

constexpr std::uint16_t operation_mode_bit(DriveOperationMode mode) noexcept {
  return static_cast<std::uint16_t>(mode);
}

constexpr std::uint16_t kKnownDriveOperationModes =
    operation_mode_bit(DriveOperationMode::ProfilePosition) |
    operation_mode_bit(DriveOperationMode::ProfileVelocity) |
    operation_mode_bit(DriveOperationMode::ProfileTorque) |
    operation_mode_bit(DriveOperationMode::CyclicSynchronousPosition) |
    operation_mode_bit(DriveOperationMode::CyclicSynchronousVelocity) |
    operation_mode_bit(DriveOperationMode::CyclicSynchronousTorque) |
    operation_mode_bit(DriveOperationMode::Homing);

constexpr DriveControlMode normalized_control_mode(
    DriveOperationMode mode) noexcept {
  switch (mode) {
    case DriveOperationMode::ProfilePosition:
    case DriveOperationMode::CyclicSynchronousPosition:
      return DriveControlMode::Position;
    case DriveOperationMode::ProfileVelocity:
    case DriveOperationMode::CyclicSynchronousVelocity:
      return DriveControlMode::Velocity;
    case DriveOperationMode::ProfileTorque:
    case DriveOperationMode::CyclicSynchronousTorque:
      return DriveControlMode::Torque;
    case DriveOperationMode::Homing:
      return DriveControlMode::Position;
  }
  return DriveControlMode::Position;
}

struct AxisCapabilities {
  std::uint8_t supported_modes{0};
  std::uint16_t supported_operation_modes{0};
  bool supports_profile_modes{false};
  bool supports_cyclic_modes{false};
  bool supports_homing{false};
  bool has_holding_brake{false};

  [[nodiscard]] bool supports(DriveControlMode mode) const noexcept {
    return (supported_modes & control_mode_bit(mode)) != 0U;
  }

  [[nodiscard]] bool supports(DriveOperationMode mode) const noexcept {
    return (supported_operation_modes & operation_mode_bit(mode)) != 0U;
  }

  [[nodiscard]] bool valid() const noexcept {
    if (supported_modes == 0U ||
        (supported_modes & static_cast<std::uint8_t>(
                               ~kKnownDriveControlModes)) != 0U ||
        (supported_operation_modes & static_cast<std::uint16_t>(
                                         ~kKnownDriveOperationModes)) != 0U) {
      return false;
    }
    for (std::uint16_t bit = 1U; bit <= operation_mode_bit(
             DriveOperationMode::Homing); bit <<= 1U) {
      if ((supported_operation_modes & bit) != 0U) {
        const auto mode = static_cast<DriveOperationMode>(bit);
        if (!supports(normalized_control_mode(mode))) return false;
      }
    }
    return true;
  }
};

template <std::size_t MaxAxes>
struct DeviceCapabilities {
  static_assert(MaxAxes > 0, "DeviceCapabilities requires positive capacity");

  std::array<AxisCapabilities, MaxAxes> axes{};
  std::size_t axis_count{0};
  bool coherent_multi_axis_state{false};
  bool synchronized_multi_axis_command{false};
  bool supports_device_clock{false};

  [[nodiscard]] bool valid() const noexcept {
    if (axis_count == 0U || axis_count > MaxAxes) return false;
    for (std::size_t i = 0; i < axis_count; ++i) {
      if (!axes[i].valid()) return false;
    }
    return true;
  }

  // Constant-time axis lookup; configuration-wide validation is performed
  // once when the device is configured, not on every real-time query.
  [[nodiscard]] bool supports(std::size_t axis,
                              DriveControlMode mode) const noexcept {
    return axis < axis_count && axis < MaxAxes && axes[axis].supports(mode);
  }
};

// Fixed-capacity identity avoids allocation and makes topology metadata safe
// to copy into real-time snapshots. Text fields must be NUL-terminated.
template <std::size_t MaxAxes>
struct DeviceIdentity {
  static_assert(MaxAxes > 0, "DeviceIdentity requires positive capacity");
  static constexpr std::size_t kTextCapacity = 32U;

  std::uint32_t vendor_id{0};
  std::uint32_t product_code{0};
  std::uint32_t revision{0};
  std::array<char, kTextCapacity> vendor_name{};
  std::array<char, kTextCapacity> product_name{};
  std::array<char, kTextCapacity> serial_number{};
  std::array<char, kTextCapacity> firmware_version{};
  std::array<std::array<char, kTextCapacity>, MaxAxes> axis_names{};
  std::size_t axis_count{0};

  [[nodiscard]] static bool text_valid(
      const std::array<char, kTextCapacity>& value,
      bool required) noexcept {
    bool terminated = false;
    bool nonempty = false;
    for (char c : value) {
      if (c == '\0') {
        terminated = true;
        break;
      }
      nonempty = true;
    }
    return terminated && (!required || nonempty);
  }

  [[nodiscard]] bool valid() const noexcept {
    if (axis_count == 0U || axis_count > MaxAxes ||
        !text_valid(vendor_name, true) ||
        !text_valid(product_name, true) ||
        !text_valid(serial_number, false) ||
        !text_valid(firmware_version, false)) {
      return false;
    }
    for (std::size_t i = 0; i < axis_count; ++i) {
      if (!text_valid(axis_names[i], true)) return false;
      for (std::size_t j = 0; j < i; ++j) {
        if (axis_names[i] == axis_names[j]) return false;
      }
    }
    return true;
  }
};

enum class HardwareClockDomain : std::uint8_t {
  Unknown,
  HostMonotonic,
  HostMonotonicRaw,
  DeviceMonotonic,
  EthercatDistributed
};

struct HardwareSampleMetadata {
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};
  HardwareClockDomain clock_domain{HardwareClockDomain::Unknown};
  bool timestamp_valid{false};
  bool coherent{false};

  [[nodiscard]] bool valid() const noexcept {
    return sequence != 0U &&
           (!timestamp_valid ||
            (timestamp_ns != 0U &&
             clock_domain != HardwareClockDomain::Unknown));
  }
};

struct HardwareCommandMetadata {
  std::uint64_t sequence{0};
  std::uint64_t deadline_ns{0};
  HardwareClockDomain clock_domain{HardwareClockDomain::Unknown};
  bool deadline_valid{false};

  [[nodiscard]] bool valid() const noexcept {
    return sequence != 0U &&
           (!deadline_valid ||
            (deadline_ns != 0U &&
             clock_domain != HardwareClockDomain::Unknown));
  }

  [[nodiscard]] bool deadline_compatible(
      HardwareClockDomain now_domain) const noexcept {
    return !deadline_valid || clock_domain == now_domain;
  }

  [[nodiscard]] bool expired(std::uint64_t now_ns,
                             HardwareClockDomain now_domain) const noexcept {
    return deadline_valid &&
           (clock_domain != now_domain || now_ns >= deadline_ns);
  }
};

}  // namespace mca::hardware
