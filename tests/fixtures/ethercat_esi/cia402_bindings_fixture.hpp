#pragma once

#include <array>
#include <cstdint>

namespace mca::test_fixtures {
struct EsiCia402PdoEntry {
  std::uint16_t index;
  std::uint8_t subindex;
  std::uint8_t bit_length;
  std::uint8_t direction;  // 0 = RxPDO/output, 1 = TxPDO/input
  std::uint8_t sync_manager;
  std::uint16_t pdo_index;
  std::uint32_t bit_offset;
};
inline constexpr std::uint32_t kVendorId = 0x000000ABU;
inline constexpr std::uint32_t kProductCode = 0x00001001U;
inline constexpr std::array<EsiCia402PdoEntry, 7U> kEsiCia402PdoEntries{{
    {0x6040U, 0U, 16U, 0U, 2U, 0x1600U, 0U},
    {0x6060U, 0U, 8U, 0U, 2U, 0x1600U, 16U},
    {0x0000U, 0U, 8U, 0U, 2U, 0x1600U, 24U},
    {0x60FFU, 0U, 32U, 0U, 2U, 0x1601U, 32U},
    {0x6041U, 0U, 16U, 1U, 3U, 0x1A00U, 0U},
    {0x6061U, 0U, 8U, 1U, 3U, 0x1A00U, 16U},
    {0x606CU, 0U, 32U, 1U, 3U, 0x1A01U, 24U},
}};
}  // namespace mca::test_fixtures
