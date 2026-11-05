#include <array>
#include <cstddef>
#include <cstdint>

#include <gtest/gtest.h>

#include "mca/hardware/cia402_pdo_bindings.hpp"
#include "mca/hardware/ethercat_pdo_mapping_profile.hpp"
#include "../fixtures/ethercat_esi/cia402_bindings_fixture.hpp"

namespace {
using namespace mca::hardware;
using Direction = EthercatPdoDirection;
struct Binding {
  std::uint16_t ring_position;
  std::uint32_t vendor_id;
  std::uint32_t product_code;
  std::uint16_t index;
  std::uint8_t subindex;
  std::uint8_t bit_length;
  std::uint16_t domain_id;
  Direction direction;
};

std::array<Binding, 10U> make_bindings() {
  // Deliberately not in PDO order: resolution must use object identity.
  return {{{0U, 0x029CU, 0x03B11002U, 0x6064U, 0U, 32U, 1U, Direction::Input},
           {0U, 0x029CU, 0x03B11002U, 0x6040U, 0U, 16U, 1U, Direction::Output},
           {0U, 0x029CU, 0x03B11002U, 0x6061U, 0U, 8U, 1U, Direction::Input},
           {0U, 0x029CU, 0x03B11002U, 0x607AU, 0U, 32U, 1U, Direction::Output},
           {0U, 0x029CU, 0x03B11002U, 0x6041U, 0U, 16U, 1U, Direction::Input},
           {0U, 0x029CU, 0x03B11002U, 0x60FFU, 0U, 32U, 1U, Direction::Output},
           {0U, 0x029CU, 0x03B11002U, 0x6060U, 0U, 8U, 1U, Direction::Output},
           {0U, 0x029CU, 0x03B11002U, 0x606CU, 0U, 32U, 1U, Direction::Input},
           {0U, 0x029CU, 0x03B11002U, 0x6071U, 0U, 16U, 1U, Direction::Output},
           {0U, 0x029CU, 0x03B11002U, 0x6077U, 0U, 16U, 1U, Direction::Input}}};
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
  EXPECT_EQ(resolved.target_torque, 8U);
  EXPECT_EQ(resolved.actual_torque, 9U);
  EXPECT_TRUE(resolved.valid());
  EXPECT_TRUE(resolved.supports(DriveOperationMode::CyclicSynchronousPosition));
  EXPECT_TRUE(resolved.supports(DriveOperationMode::CyclicSynchronousVelocity));
  EXPECT_TRUE(resolved.supports(DriveOperationMode::CyclicSynchronousTorque));
  EXPECT_FALSE(resolved.supports(DriveOperationMode::Homing));
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
      bindings, 6U, 0U, 0x029CU, 0x03B11002U, resolved));
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

TEST(Cia402PdoBindings, AcceptsDriveWithOnlyMandatoryAndVelocityPdos) {
  const std::array<Binding, 6U> bindings{{
      {0U, 0x1234U, 0x5678U, 0x6040U, 0U, 16U, 1U, Direction::Output},
      {0U, 0x1234U, 0x5678U, 0x6060U, 0U, 8U, 1U, Direction::Output},
      {0U, 0x1234U, 0x5678U, 0x6041U, 0U, 16U, 1U, Direction::Input},
      {0U, 0x1234U, 0x5678U, 0x6061U, 0U, 8U, 1U, Direction::Input},
      {0U, 0x1234U, 0x5678U, 0x60FFU, 0U, 32U, 1U, Direction::Output},
      {0U, 0x1234U, 0x5678U, 0x606CU, 0U, 32U, 1U, Direction::Input}}};
  Cia402PdoBindings resolved{};
  ASSERT_TRUE(resolve_cia402_pdo_bindings(
      bindings, bindings.size(), 0U, 0x1234U, 0x5678U, resolved));
  EXPECT_TRUE(resolved.valid());
  EXPECT_EQ(resolved.target_position, Cia402PdoBindings::kInvalid);
  EXPECT_EQ(resolved.actual_position, Cia402PdoBindings::kInvalid);
  EXPECT_TRUE(resolved.supports(DriveOperationMode::CyclicSynchronousVelocity));
  EXPECT_FALSE(resolved.supports(DriveOperationMode::CyclicSynchronousPosition));
  EXPECT_FALSE(resolved.supports(DriveOperationMode::CyclicSynchronousTorque));
}

TEST(Cia402PdoBindings, RejectsMalformedOptionalModeSpecificPdo) {
  auto bindings = make_bindings();
  bindings[8].bit_length = 32U;
  Cia402PdoBindings resolved{};
  EXPECT_FALSE(resolve_cia402_pdo_bindings(
      bindings, bindings.size(), 0U, 0x029CU, 0x03B11002U, resolved));
}

TEST(Cia402PdoBindings, MatchesPythonNormalizedEsiFixtureAcrossCia402AndPdoProfile) {
  using Fixture = mca::test_fixtures::EsiCia402PdoEntry;
  constexpr auto& fixture = mca::test_fixtures::kEsiCia402PdoEntries;
  std::array<Binding, fixture.size()> bindings{};
  using Profile = EthercatPdoMappingProfile<fixture.size()>;
  Profile profile{};

  for (std::size_t i = 0; i < fixture.size(); ++i) {
    const Fixture& source = fixture[i];
    const Direction direction =
        source.direction == 0U ? Direction::Output : Direction::Input;
    bindings[i] = {0U, mca::test_fixtures::kVendorId,
                   mca::test_fixtures::kProductCode, source.index,
                   source.subindex, source.bit_length, 1U, direction};
    ASSERT_TRUE(profile.add({0U, mca::test_fixtures::kVendorId,
                             mca::test_fixtures::kProductCode,
                             source.sync_manager, source.pdo_index,
                             source.index, source.subindex,
                             source.bit_length, direction}))
        << "fixture entry " << i;
  }

  ASSERT_TRUE(profile.valid());
  ASSERT_EQ(profile.size(), fixture.size());
  Cia402PdoBindings resolved{};
  ASSERT_TRUE(resolve_cia402_pdo_bindings(
      bindings, bindings.size(), 0U, mca::test_fixtures::kVendorId,
      mca::test_fixtures::kProductCode, resolved));

  EXPECT_EQ(resolved.controlword, 0U);
  EXPECT_EQ(resolved.mode_of_operation, 1U);
  EXPECT_EQ(resolved.target_velocity, 3U);
  EXPECT_EQ(resolved.statusword, 4U);
  EXPECT_EQ(resolved.mode_display, 5U);
  EXPECT_EQ(resolved.actual_velocity, 6U);
  EXPECT_EQ(resolved.target_position, Cia402PdoBindings::kInvalid);
  EXPECT_EQ(resolved.actual_position, Cia402PdoBindings::kInvalid);

  for (std::size_t i = 0; i < fixture.size(); ++i) {
    EXPECT_EQ(profile[i].index, fixture[i].index);
    EXPECT_EQ(profile[i].subindex, fixture[i].subindex);
    EXPECT_EQ(profile[i].bit_length, fixture[i].bit_length);
    EXPECT_EQ(profile[i].sync_manager, fixture[i].sync_manager);
    EXPECT_EQ(profile[i].pdo_index, fixture[i].pdo_index);
    EXPECT_EQ(profile[i].direction,
              fixture[i].direction == 0U ? Direction::Output : Direction::Input);
  }
  EXPECT_EQ(fixture[0].bit_offset, 0U);
  EXPECT_EQ(fixture[1].bit_offset, 16U);
  EXPECT_EQ(fixture[2].bit_offset, 24U);
  EXPECT_EQ(fixture[3].bit_offset, 32U);
  EXPECT_EQ(fixture[4].bit_offset, 0U);
  EXPECT_EQ(fixture[5].bit_offset, 16U);
  EXPECT_EQ(fixture[6].bit_offset, 24U);
}
}  // namespace
