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

  [[nodiscard]] constexpr bool valid() const noexcept {
    const std::array<std::size_t, 8U> values{
        controlword, target_position, target_velocity, mode_of_operation,
        statusword, actual_position, actual_velocity, mode_display};
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (values[i] == kInvalid) return false;
      for (std::size_t j = 0; j < i; ++j)
        if (values[i] == values[j]) return false;
    }
    return true;
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
                        std::size_t& result) noexcept {
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
    return matches == 1U;
  };

  if (!find(0x6040U, 0U, 16U, Direction::Output, resolved.controlword) ||
      !find(0x607AU, 0U, 32U, Direction::Output, resolved.target_position) ||
      !find(0x60FFU, 0U, 32U, Direction::Output, resolved.target_velocity) ||
      !find(0x6060U, 0U, 8U, Direction::Output, resolved.mode_of_operation) ||
      !find(0x6041U, 0U, 16U, Direction::Input, resolved.statusword) ||
      !find(0x6064U, 0U, 32U, Direction::Input, resolved.actual_position) ||
      !find(0x606CU, 0U, 32U, Direction::Input, resolved.actual_velocity) ||
      !find(0x6061U, 0U, 8U, Direction::Input, resolved.mode_display) ||
      !resolved.valid()) return false;

  output = resolved;
  return true;
}

}  // namespace mca::hardware
