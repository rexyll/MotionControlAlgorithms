#pragma once

#include "cia402_bindings_fixture.hpp"

namespace mca::test_fixtures {
inline constexpr std::array<EsiCia402PdoEntry, 11U> kEsiCia402AllModesEntries{{
    {0x6040U, 0U, 16U, 0U, 2U, 0x1600U, 0U},
    {0x6060U, 0U, 8U, 0U, 2U, 0x1600U, 16U},
    {0x0000U, 0U, 8U, 0U, 2U, 0x1600U, 24U},
    {0x60FFU, 0U, 32U, 0U, 2U, 0x1601U, 32U},
    {0x607AU, 0U, 32U, 0U, 2U, 0x1602U, 64U},
    {0x6071U, 0U, 16U, 0U, 2U, 0x1602U, 96U},
    {0x6041U, 0U, 16U, 1U, 3U, 0x1A00U, 0U},
    {0x6061U, 0U, 8U, 1U, 3U, 0x1A00U, 16U},
    {0x606CU, 0U, 32U, 1U, 3U, 0x1A01U, 24U},
    {0x6064U, 0U, 32U, 1U, 3U, 0x1A02U, 56U},
    {0x6077U, 0U, 16U, 1U, 3U, 0x1A02U, 88U},
}};
}  // namespace mca::test_fixtures
