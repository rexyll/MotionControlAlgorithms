#pragma once

#include <cstddef>
#include <cstdint>

namespace mca::hardware {

// EtherCAT PDO bit offsets are LSB-first within each byte. The helpers are
// deliberately allocation-free and bounded to 64 bits so an adapter can use
// them directly on an activated process-data image from a cyclic path.
// The bounds check is performed before any image byte is touched.
constexpr bool ethercat_read_pdo_bits(const std::uint8_t* data,
                                      std::size_t byte_size,
                                      std::uint32_t bit_offset,
                                      std::uint8_t bit_length,
                                      std::uint64_t& value) noexcept {
  if (data == nullptr || bit_length == 0U || bit_length > 64U)
    return false;
  const std::uint64_t end = static_cast<std::uint64_t>(bit_offset) + bit_length;
  const std::uint64_t required_bytes = (end + 7U) / 8U;
  if (required_bytes > static_cast<std::uint64_t>(byte_size))
    return false;

  std::uint64_t result = 0U;
  for (std::uint8_t bit = 0U; bit < bit_length; ++bit) {
    const std::uint64_t absolute = static_cast<std::uint64_t>(bit_offset) + bit;
    const std::uint8_t byte = data[static_cast<std::size_t>(absolute / 8U)];
    if ((byte & static_cast<std::uint8_t>(1U << (absolute % 8U))) != 0U)
      result |= (std::uint64_t{1U} << bit);
  }
  value = result;
  return true;
}

constexpr bool ethercat_write_pdo_bits(std::uint8_t* data,
                                       std::size_t byte_size,
                                       std::uint32_t bit_offset,
                                       std::uint8_t bit_length,
                                       std::uint64_t value) noexcept {
  if (data == nullptr || bit_length == 0U || bit_length > 64U)
    return false;
  const std::uint64_t end = static_cast<std::uint64_t>(bit_offset) + bit_length;
  const std::uint64_t required_bytes = (end + 7U) / 8U;
  if (required_bytes > static_cast<std::uint64_t>(byte_size))
    return false;
  if (bit_length < 64U && (value >> bit_length) != 0U)
    return false;

  for (std::uint8_t bit = 0U; bit < bit_length; ++bit) {
    const std::uint64_t absolute = static_cast<std::uint64_t>(bit_offset) + bit;
    auto& byte = data[static_cast<std::size_t>(absolute / 8U)];
    const std::uint8_t mask = static_cast<std::uint8_t>(1U << (absolute % 8U));
    if ((value & (std::uint64_t{1U} << bit)) != 0U)
      byte = static_cast<std::uint8_t>(byte | mask);
    else
      byte = static_cast<std::uint8_t>(byte & static_cast<std::uint8_t>(~mask));
  }
  return true;
}

}  // namespace mca::hardware
