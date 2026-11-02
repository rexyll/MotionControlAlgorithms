#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace mca::hardware {

// Drive control capabilities are reported per axis. These values describe
// normalized command semantics; protocol-specific modes (PPM/CSP/etc.) remain
// an adapter concern and must preserve their timing semantics.
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

struct AxisCapabilities {
  std::uint8_t supported_modes{0};
  bool supports_profile_modes{false};
  bool supports_cyclic_modes{false};
  bool supports_homing{false};
  bool has_holding_brake{false};

  [[nodiscard]] bool supports(DriveControlMode mode) const noexcept {
    return (supported_modes & control_mode_bit(mode)) != 0U;
  }

  [[nodiscard]] bool valid() const noexcept {
    return supported_modes != 0U &&
           (supported_modes & static_cast<std::uint8_t>(
                                  ~kKnownDriveControlModes)) == 0U &&
           (!supports_profile_modes || supported_modes != 0U) &&
           (!supports_cyclic_modes || supported_modes != 0U);
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

  [[nodiscard]] bool supports(std::size_t axis,
                              DriveControlMode mode) const noexcept {
    return valid() && axis < axis_count && axes[axis].supports(mode);
  }
};

// Clock domains are intentionally explicit. Host monotonic time must not be
// compared directly with a device or EtherCAT distributed-clock timestamp.
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

  // A clock-domain mismatch fails closed: the caller must not submit a
  // deadline-constrained command when it cannot compare the clocks safely.
  [[nodiscard]] bool expired(std::uint64_t now_ns,
                             HardwareClockDomain now_domain) const noexcept {
    return deadline_valid &&
           (clock_domain != now_domain || now_ns >= deadline_ns);
  }
};

}  // namespace mca::hardware
