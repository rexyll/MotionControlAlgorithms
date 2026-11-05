#include <array>
#include <cstdint>
#include <iostream>

#include "mca/hardware/ethercat_pdo_image.hpp"

int main() {
  std::array<std::uint8_t, 8> image{};
  constexpr std::uint64_t value = 0xFEDCBA9876543210ULL;

  if (!mca::hardware::ethercat_write_pdo_bits(image.data(), image.size(), 0U, 64U, value)) return 1;

  std::uint64_t round_trip = 0U;
  if (!mca::hardware::ethercat_read_pdo_bits(image.data(), image.size(), 0U, 64U, round_trip)) return 2;
  if (round_trip != value) return 3;

  const auto before_reject = image;
  if (mca::hardware::ethercat_write_pdo_bits(image.data(), image.size(), 3U, 5U, 0x20U)) return 4;
  if (image != before_reject) return 5;

  if (mca::hardware::ethercat_write_pdo_bits(image.data(), image.size(), 59U, 6U, 0x3FU)) return 6;
  if (mca::hardware::ethercat_read_pdo_bits(image.data(), image.size(), 59U, 10U, round_trip)) return 7;
  if (round_trip != value) return 8;

  std::array<std::uint8_t, 3> unaligned{0xA5U, 0x5AU, 0xC3U};
  if (!mca::hardware::ethercat_write_pdo_bits(unaligned.data(), unaligned.size(), 3U, 11U, 0x2A5U)) return 9;
  if (!mca::hardware::ethercat_read_pdo_bits(unaligned.data(), unaligned.size(), 3U, 11U, round_trip)) return 10;
  if (round_trip != 0x2A5U) return 11;
  if ((unaligned[1] & 0xC0U) != 0x40U || (unaligned[0] & 0x07U) != 0x05U || unaligned[2] != 0xC3U) return 12;

  std::array<std::uint8_t, 1> final_byte{0x2AU};
  if (!mca::hardware::ethercat_write_pdo_bits(
          final_byte.data(), final_byte.size(), 7U, 1U, 1U)) return 13;
  if (final_byte[0] != 0xAAU) return 14;
  if (!mca::hardware::ethercat_read_pdo_bits(
          final_byte.data(), final_byte.size(), 7U, 1U, round_trip)) return 15;
  if (round_trip != 1U) return 16;
  if (!mca::hardware::ethercat_write_pdo_bits(
          final_byte.data(), final_byte.size(), 7U, 1U, 0U)) return 17;
  if (final_byte[0] != 0x2AU) return 18;

  const auto before_final_byte_reject = final_byte;
  if (mca::hardware::ethercat_write_pdo_bits(
          final_byte.data(), final_byte.size(), 7U, 2U, 0U)) return 19;
  if (final_byte != before_final_byte_reject) return 20;
  round_trip = 0x55U;
  if (mca::hardware::ethercat_read_pdo_bits(
          final_byte.data(), final_byte.size(), 7U, 2U, round_trip)) return 21;
  if (round_trip != 0x55U) return 22;

  if (mca::hardware::ethercat_read_pdo_bits(
          final_byte.data(), final_byte.size(), UINT32_MAX, 1U, round_trip)) return 23;
  if (mca::hardware::ethercat_write_pdo_bits(
          final_byte.data(), final_byte.size(), UINT32_MAX, 1U, 0U)) return 24;
  if (final_byte != before_final_byte_reject) return 25;

  std::cout << "EtherCAT PDO image smoke passed\n";
  return 0;
}
