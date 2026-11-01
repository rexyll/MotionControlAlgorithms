#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "mca/core/types.hpp"

namespace mca::hardware {

// Validity is explicit: a driver must not report an unavailable sensor field
// as a measured zero. Fields not selected in valid_fields are ignored by
// JointState::valid().
enum class JointStateField : std::uint8_t {
  Position = 1U << 0U,
  Velocity = 1U << 1U,
  Acceleration = 1U << 2U,
  Effort = 1U << 3U
};

template <typename Scalar, std::size_t Dimensions>
struct JointState {
  static_assert(Dimensions > 0, "JointState requires at least one dimension");

  static constexpr std::uint8_t kPositionValid =
      static_cast<std::uint8_t>(JointStateField::Position);
  static constexpr std::uint8_t kVelocityValid =
      static_cast<std::uint8_t>(JointStateField::Velocity);
  static constexpr std::uint8_t kAccelerationValid =
      static_cast<std::uint8_t>(JointStateField::Acceleration);
  static constexpr std::uint8_t kEffortValid =
      static_cast<std::uint8_t>(JointStateField::Effort);
  static constexpr std::uint8_t kKnownFields =
      kPositionValid | kVelocityValid | kAccelerationValid | kEffortValid;

  std::array<Scalar, Dimensions> position{};
  std::array<Scalar, Dimensions> velocity{};
  std::array<Scalar, Dimensions> acceleration{};
  std::array<Scalar, Dimensions> effort{};
  std::uint64_t sequence{0};
  std::uint8_t valid_fields{0};

  [[nodiscard]] bool has(JointStateField field) const noexcept {
    return (valid_fields & static_cast<std::uint8_t>(field)) != 0U;
  }

  [[nodiscard]] bool valid() const noexcept {
    if (valid_fields == 0U ||
        (valid_fields & static_cast<std::uint8_t>(~kKnownFields)) != 0U) {
      return false;
    }
    const auto finite_if_valid = [](const auto& values) noexcept {
      for (const auto value : values) {
        if (!std::isfinite(value)) return false;
      }
      return true;
    };
    return (!has(JointStateField::Position) || finite_if_valid(position)) &&
           (!has(JointStateField::Velocity) || finite_if_valid(velocity)) &&
           (!has(JointStateField::Acceleration) || finite_if_valid(acceleration)) &&
           (!has(JointStateField::Effort) || finite_if_valid(effort));
  }
};

template <typename Scalar, std::size_t Dimensions>
struct JointEffortCommand {
  static_assert(Dimensions > 0,
                "JointEffortCommand requires at least one dimension");

  std::array<Scalar, Dimensions> effort{};
  std::uint64_t sequence{0};

  [[nodiscard]] bool valid() const noexcept {
    for (const auto value : effort) {
      if (!std::isfinite(value)) return false;
    }
    return true;
  }
};

enum class HardwareLifecycleState : std::uint8_t {
  Uninitialized,
  Inactive,
  Active,
  Faulted
};

// Device lifecycle is separate from state and command capabilities. Concrete
// drivers own configuration parsing, transport threads and vendor resources.
class IHardwareLifecycle {
 public:
  virtual ~IHardwareLifecycle() = default;

  // Success transitions Uninitialized -> Inactive. Implementations report
  // initialization/connection failures as a non-Ok status.
  virtual core::Status initialize() noexcept = 0;

  // Activation is allowed only from Inactive.
  virtual core::Status activate() noexcept = 0;

  // Deactivation must inhibit further command output. Failure must remain
  // observable; callers must not assume the device is safe after an error.
  virtual core::Status deactivate() noexcept = 0;

  [[nodiscard]] virtual HardwareLifecycleState lifecycle_state() const noexcept = 0;
};

// A state source returns one internally coherent sample. On any non-Ok result,
// output must remain unchanged. sequence is monotonically increasing per driver
// instance and is intended for detecting dropped/repeated samples.
template <typename Scalar, std::size_t Dimensions>
class IJointStateSource {
 public:
  using State = JointState<Scalar, Dimensions>;

  virtual ~IJointStateSource() = default;
  virtual core::Status read_state(State& output) noexcept = 0;
};

// An effort sink accepts fixed-size commands without prescribing a transport.
// Invalid commands must be rejected before hardware I/O. A failed multi-axis
// write may have device-specific partial-application semantics; concrete
// drivers must document that behavior and callers must treat failures as a
// control fault. request_safe_stop() is a software request, not a substitute
// for a safety-rated, independently wired emergency-stop circuit.
template <typename Scalar, std::size_t Dimensions>
class IJointEffortCommandSink {
 public:
  using Command = JointEffortCommand<Scalar, Dimensions>;

  virtual ~IJointEffortCommandSink() = default;
  virtual core::Status write_effort(const Command& command) noexcept = 0;
  virtual core::Status request_safe_stop() noexcept = 0;
};

using ScalarJointState = JointState<double, 1>;
using ScalarJointEffortCommand = JointEffortCommand<double, 1>;

}  // namespace mca::hardware
