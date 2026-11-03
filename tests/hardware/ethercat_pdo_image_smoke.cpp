#include <array>
#include <cstdint>
#include <iostream>

#include "mca/hardware/ethercat_pdo_image.hpp"

int main() {
  std::array<std::uint8_t, 8> image{};
  constexpr std::uint64_t value = 0xFEDCBA9876543210ULL;

  if (!mca::hardware::ethercat_write_pdo_bits(image.data(), image.size(), 3U, 64U, value)) return 1;

  std::uint64_t round_trip = 0U;
  if (!mca::hardware::ethercat_read_pdo_bits(image.data(), image.size(), 3U, 64U, round_trip)) return 2;
  if (round_trip != value) return 3;

  if (mca::hardware::ethercat_write_pdo_bits(image.data(), image.size(), 59U, 6U, 0x3FU)) return 4;
  if (mca::hardware::ethercat_read_pdo_bits(image.data(), image.size(), 59U, 10U, round_trip)) return 5;
  if (round_trip != value) return 6;

  std::cout << "EtherCAT PDO image smoke passed\n";
  return 0;
}
