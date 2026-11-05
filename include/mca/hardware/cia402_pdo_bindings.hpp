#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "mca/hardware/ethercat_adapter_state.hpp"

namespace mca::hardware {

// Binding indices are adapter process-image indices, not PDO order or object
// dictionary indices. Resolve every CiA 402 object by its complete identity.
struct Cia402PdoBindings {
  static constexpr std::size_t kInvalid =
      std::numeric_limits<std::size_t>::max();

  std::size_t controlword{kInvalid};       // 0x6040:00, RxPDO, UNSIGNED16
  std::size_t target_position{kInvalid};   // 0x607A:00, RxPDO, INTEGER32
  std::size_t target_velocity{kInvalid};   // 0x60FF:00, RxPDO, INTEGER32
  std::size_t mode_of_operation{kInvalid}; // 0x6060:00, RxPDO, INTEGER8
  std::size_t statusword{kInvalid};        // 0x6041:00, TxPDO, UNSIGNED16
  std::size_t actual_position{kInvalid};   // 0x6064:00, TxPDO, INTEGER32
  std::size_t actual_velocity{kInvalid};   // 0x606C:00, TxPDO, INTEGER32
  std::size_t mode_display{kInvalid};      // 0x6061:00, TxPDO, INTEGER8
  std::size_t target_torque{kInvalid};     // 0x6071:00, RxPDO, INTEGER16
  std::size_t actual_torque{kInvalid};     // 0x6077:00, TxPDO, INTEGER16

  // Controlword/statusword and mode command/feedback form the minimum cyclic
  // CiA 402 binding. Mode-specific setpoint/feedback pairs are optional and
  // must be checked before enabling that mode.
  [[nodiscard]] constexpr bool valid() const noexcept {
    if (controlword == kInvalid || statusword == kInvalid ||
        mode_of_operation == kInvalid || mode_display == kInvalid)
      return false;
    const std::array<std::size_t, 10U> values{
        controlword, target_position, target_velocity, mode_of_operation,
        statusword, actual_position, actual_velocity, mode_display,
        target_torque, actual_torque};
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (values[i] == kInvalid) continue;
      for (std::size_t j = 0; j < i; ++j)
        if (values[i] == values[j]) return false;
    }
    return true;
  }

  [[nodiscard]] constexpr bool supports(DriveOperationMode mode) const noexcept {
    if (!valid()) return false;
    switch (mode) {
      case DriveOperationMode::ProfilePosition:
      case DriveOperationMode::CyclicSynchronousPosition:
        return target_position != kInvalid && actual_position != kInvalid;
      case DriveOperationMode::ProfileVelocity:
      case DriveOperationMode::CyclicSynchronousVelocity:
        return target_velocity != kInvalid && actual_velocity != kInvalid;
      case DriveOperationMode::ProfileTorque:
      case DriveOperationMode::CyclicSynchronousTorque:
        return target_torque != kInvalid && actual_torque != kInvalid;
      case DriveOperationMode::Homing:
        return false;  // Requires a dedicated homing profile/binding.
    }
    return false;
  }
};

template <typename Binding, std::size_t MaxEntries>
[[nodiscard]] bool resolve_cia402_pdo_bindings(
    const std::array<Binding, MaxEntries>& bindings,
    std::size_t binding_count, std::uint16_t ring_position,
    std::uint32_t vendor_id, std::uint32_t product_code,
    Cia402PdoBindings& output) noexcept {
  using Direction = EthercatPdoDirection;
  if (binding_count == 0U || binding_count > MaxEntries) return false;

  Cia402PdoBindings resolved{};
  const auto find = [&](std::uint16_t index, std::uint8_t subindex,
                        std::uint8_t width, Direction direction,
                        std::size_t& result, bool required) noexcept {
    std::size_t matches = 0U;
    for (std::size_t i = 0; i < binding_count; ++i) {
      const Binding& item = bindings[i];
      if (item.ring_position != ring_position || item.vendor_id != vendor_id ||
          item.product_code != product_code || item.index != index ||
          item.subindex != subindex) continue;
      if (item.bit_length != width || item.direction != direction) return false;
      result = i;
      ++matches;
    }
    return required ? matches == 1U : matches <= 1U;
  };

  if (!find(0x6040U, 0U, 16U, Direction::Output, resolved.controlword, true) ||
      !find(0x6060U, 0U, 8U, Direction::Output, resolved.mode_of_operation, true) ||
      !find(0x6041U, 0U, 16U, Direction::Input, resolved.statusword, true) ||
      !find(0x6061U, 0U, 8U, Direction::Input, resolved.mode_display, true) ||
      !find(0x607AU, 0U, 32U, Direction::Output, resolved.target_position, false) ||
      !find(0x60FFU, 0U, 32U, Direction::Output, resolved.target_velocity, false) ||
      !find(0x6064U, 0U, 32U, Direction::Input, resolved.actual_position, false) ||
      !find(0x606CU, 0U, 32U, Direction::Input, resolved.actual_velocity, false) ||
      !find(0x6071U, 0U, 16U, Direction::Output, resolved.target_torque, false) ||
      !find(0x6077U, 0U, 16U, Direction::Input, resolved.actual_torque, false) ||
      !resolved.valid()) return false;

  output = resolved;
  return true;
}

}  // namespace mca::hardware
