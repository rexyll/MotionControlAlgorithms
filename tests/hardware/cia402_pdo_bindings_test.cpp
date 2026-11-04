#include <array>
#include <cstddef>
#include <cstdint>

#include <gtest/gtest.h>

#include "mca/hardware/cia402_pdo_bindings.hpp"

namespace {
using namespace mca::hardware;
using Adapter = EthercatIghAdapter<16U, 4U, 16U>;
using Binding = Adapter::PdoBinding;
using Direction = EthercatPdoDirection;

std::array<Binding, 8U> make_bindings() {
  // Deliberately not in PDO order: resolution must use object identity.
  return {{{0U, 0x029CU, 0x03B11002U, 0x6064U, 0U, 32U, 1U, Direction::Input},
           {0U, 0x029CU, 0x03B11002U, 0x6040U, 0U, 16U, 1U, Direction::Output},
           {0U, 0x029CU, 0x03B11002U, 0x6061U, 0U, 8U, 1U, Direction::Input},
           {0U, 0x029CU, 0x03B11002U, 0x607AU, 0U, 32U, 1U, Direction::Output},
           {0U, 0x029CU, 0x03B11002U, 0x6041U, 0U, 16U, 1U, Direction::Input},
           {0U, 0x029CU, 0x03B11002U, 0x60FFU, 0U, 32U, 1U, Direction::Output},
           {0U, 0x029CU, 0x03B11002U, 0x6060U, 0U, 8U, 1U, Direction::Output},
           {0U, 0x029CU, 0x03B11002U, 0x606CU, 0U, 32U, 1U, Direction::Input}}};
}

TEST(Cia402PdoBindings, ResolvesAllObjectsByIndexAndDirection) {
  const auto bindings = make_bindings();
  Cia402PdoBindings resolved{};
  ASSERT_TRUE(resolve_cia402_pdo_bindings(
      bindings, bindings.size(), 0U, 0x029CU, 0x03B11002U, resolved));
  EXPECT_EQ(resolved.controlword, 1U);
  EXPECT_EQ(resolved.target_position, 3U);
  EXPECT_EQ(resolved.target_velocity, 5U);
  EXPECT_EQ(resolved.mode_of_operation, 6U);
  EXPECT_EQ(resolved.statusword, 4U);
  EXPECT_EQ(resolved.actual_position, 0U);
  EXPECT_EQ(resolved.actual_velocity, 7U);
  EXPECT_EQ(resolved.mode_display, 2U);
  EXPECT_TRUE(resolved.valid());
}

TEST(Cia402PdoBindings, RejectsWrongWidthDirectionIdentityAndMissingObjects) {
  auto bindings = make_bindings();
  Cia402PdoBindings resolved{};
  bindings[1].bit_length = 32U;
  EXPECT_FALSE(resolve_cia402_pdo_bindings(
      bindings, bindings.size(), 0U, 0x029CU, 0x03B11002U, resolved));

  bindings = make_bindings();
  bindings[1].direction = Direction::Input;
  EXPECT_FALSE(resolve_cia402_pdo_bindings(
      bindings, bindings.size(), 0U, 0x029CU, 0x03B11002U, resolved));

  bindings = make_bindings();
  EXPECT_FALSE(resolve_cia402_pdo_bindings(
      bindings, bindings.size() - 1U, 0U, 0x029CU, 0x03B11002U, resolved));
  EXPECT_FALSE(resolve_cia402_pdo_bindings(
      bindings, bindings.size(), 0U, 0x029CU, 0xDEADBEEFU, resolved));
}

TEST(Cia402PdoBindings, RejectsDuplicateObjectEvenWhenAllOtherFieldsMatch) {
  auto bindings = make_bindings();
  bindings[7] = bindings[0];
  Cia402PdoBindings resolved{};
  EXPECT_FALSE(resolve_cia402_pdo_bindings(
      bindings, bindings.size(), 0U, 0x029CU, 0x03B11002U, resolved));
}
}  // namespace
