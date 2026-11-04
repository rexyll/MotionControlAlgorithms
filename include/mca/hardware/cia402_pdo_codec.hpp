#pragma once

#include <cstddef>
#include <cstdint>

#include "mca/hardware/ethercat_pdo_image.hpp"

namespace mca::hardware {

// CiA 402 scalar objects use little-endian PDO bit fields. These helpers
// deliberately require the exact object width so a misconfigured ESI/PDO map
// cannot silently truncate a controlword, mode, or signed setpoint.
constexpr bool cia402_read_u8(const std::uint8_t* image, std::size_t bytes,
                              std::uint32_t bit_offset,
                              std::uint8_t& value) noexcept {
  std::uint64_t raw = 0U;
  if (!ethercat_read_pdo_bits(image, bytes, bit_offset, 8U, raw)) return false;
  value = static_cast<std::uint8_t>(raw);
  return true;
}

constexpr bool cia402_read_i8(const std::uint8_t* image, std::size_t bytes,
                              std::uint32_t bit_offset,
                              std::int8_t& value) noexcept {
  std::uint8_t raw = 0U;
  if (!cia402_read_u8(image, bytes, bit_offset, raw)) return false;
  const std::int16_t signed_value = (raw & 0x80U) != 0U
      ? static_cast<std::int16_t>(raw) - 256
      : static_cast<std::int16_t>(raw);
  value = static_cast<std::int8_t>(signed_value);
  return true;
}

constexpr bool cia402_read_u16(const std::uint8_t* image, std::size_t bytes,
                               std::uint32_t bit_offset,
                               std::uint16_t& value) noexcept {
  std::uint64_t raw = 0U;
  if (!ethercat_read_pdo_bits(image, bytes, bit_offset, 16U, raw)) return false;
  value = static_cast<std::uint16_t>(raw);
  return true;
}

constexpr bool cia402_read_i32(const std::uint8_t* image, std::size_t bytes,
                               std::uint32_t bit_offset,
                               std::int32_t& value) noexcept {
  std::uint64_t raw = 0U;
  if (!ethercat_read_pdo_bits(image, bytes, bit_offset, 32U, raw)) return false;
  const std::uint32_t bits = static_cast<std::uint32_t>(raw);
  const std::int64_t signed_value = (bits & 0x80000000U) != 0U
      ? static_cast<std::int64_t>(bits) - 0x100000000LL
      : static_cast<std::int64_t>(bits);
  value = static_cast<std::int32_t>(signed_value);
  return true;
}

constexpr bool cia402_write_u8(std::uint8_t* image, std::size_t bytes,
                               std::uint32_t bit_offset,
                               std::uint8_t value) noexcept {
  return ethercat_write_pdo_bits(image, bytes, bit_offset, 8U, value);
}

constexpr bool cia402_write_i8(std::uint8_t* image, std::size_t bytes,
                               std::uint32_t bit_offset,
                               std::int8_t value) noexcept {
  return ethercat_write_pdo_bits(
      image, bytes, bit_offset, 8U,
      static_cast<std::uint8_t>(value));
}

constexpr bool cia402_write_u16(std::uint8_t* image, std::size_t bytes,
                                std::uint32_t bit_offset,
                                std::uint16_t value) noexcept {
  return ethercat_write_pdo_bits(image, bytes, bit_offset, 16U, value);
}

constexpr bool cia402_write_i32(std::uint8_t* image, std::size_t bytes,
                                std::uint32_t bit_offset,
                                std::int32_t value) noexcept {
  return ethercat_write_pdo_bits(
      image, bytes, bit_offset, 32U,
      static_cast<std::uint32_t>(value));
}

}  // namespace mca::hardware
