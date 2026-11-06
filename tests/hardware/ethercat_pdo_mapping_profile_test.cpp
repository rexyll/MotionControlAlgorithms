#include <gtest/gtest.h>

#include "mca/hardware/ethercat_pdo_mapping_profile.hpp"

namespace {
using Profile = mca::hardware::EthercatPdoMappingProfile<8U>;
using Direction = mca::hardware::EthercatPdoDirection;

TEST(EthercatPdoMappingProfile, AcceptsOrderedMixedWidthEntriesAcrossSyncManagers) {
  Profile profile{};
  ASSERT_TRUE(profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6040U, 0U, 16U, Direction::Output}));
  ASSERT_TRUE(profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x607aU, 0U, 32U, Direction::Output}));
  ASSERT_TRUE(profile.add({5U, 0x29cU, 0x03b11002U, 3U, 0x1a00U, 0x6041U, 0U, 16U, Direction::Input}));
  EXPECT_TRUE(profile.valid());
  EXPECT_EQ(profile.size(), 3U);
}

TEST(EthercatPdoMappingProfile, RejectsDuplicateObjectEntries) {
  Profile profile{};
  ASSERT_TRUE(profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6040U, 0U, 16U, Direction::Output}));
  EXPECT_FALSE(profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6040U, 0U, 16U, Direction::Output}));
}

TEST(EthercatPdoMappingProfile, RejectsSyncManagerOrderRegression) {
  Profile profile{};
  ASSERT_TRUE(profile.add({5U, 0x29cU, 0x03b11002U, 3U, 0x1a00U, 0x6041U, 0U, 16U, Direction::Input}));
  EXPECT_FALSE(profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6040U, 0U, 16U, Direction::Output}));
}

TEST(EthercatPdoMappingProfile, RejectsMixedDirectionWithinSyncManager) {
  Profile profile{};
  ASSERT_TRUE(profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6040U, 0U, 16U, Direction::Output}));
  EXPECT_FALSE(profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6041U, 0U, 16U, Direction::Input}));
}

TEST(EthercatPdoMappingProfile, RejectsZeroWidthButAcceptsWideUnboundEntries) {
  Profile profile{};
  EXPECT_FALSE(profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6040U, 0U, 0U, Direction::Output}));
  ASSERT_TRUE(profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6040U, 0U, 128U, Direction::Output}));
  EXPECT_TRUE(profile.valid());
}

TEST(EthercatPdoMappingProfile, AcceptsRepeatedPaddingEntriesAndRejectsMalformedPadding) {
  Profile profile{};
  ASSERT_TRUE(profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x0000U, 0U, 8U, Direction::Output}));
  ASSERT_TRUE(profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x0000U, 0U, 8U, Direction::Output}));
  ASSERT_TRUE(profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6040U, 0U, 16U, Direction::Output}));
  EXPECT_TRUE(profile.valid());

  Profile malformed{};
  EXPECT_FALSE(malformed.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x0000U, 1U, 8U, Direction::Output}));
}

TEST(EthercatPdoMappingProfile, AcceptsMultipleOrderedPdosWithinOneSyncManager) {
  Profile profile;
  ASSERT_TRUE(profile.add({0U, 0x2U, 0x1234U, 2U, 0x1600U,
      0x6040U, 0U, 16U, Direction::Output}));
  ASSERT_TRUE(profile.add({0U, 0x2U, 0x1234U, 2U, 0x1600U,
      0x607AU, 0U, 32U, Direction::Output}));
  ASSERT_TRUE(profile.add({0U, 0x2U, 0x1234U, 2U, 0x1601U,
      0x60FFU, 0U, 32U, Direction::Output}));
  EXPECT_TRUE(profile.valid());
  EXPECT_EQ(profile.size(), 3U);
}

TEST(EthercatPdoMappingProfile, RejectsReappearingClosedPdoGroup) {
  Profile profile{};
  ASSERT_TRUE(profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6040U, 0U, 16U, Direction::Output}));
  ASSERT_TRUE(profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1601U, 0x607aU, 0U, 32U, Direction::Output}));
  EXPECT_FALSE(profile.add({5U, 0x29cU, 0x03b11002U, 2U, 0x1600U, 0x6060U, 0U, 8U, Direction::Output}));
}
}  // namespace


#include "mca/hardware/ethercat_pdo_readback.hpp"

namespace {
using Readback = mca::hardware::EthercatPdoReadback<8U>;
using ReadbackRecord = mca::hardware::EthercatPdoReadbackRecord;
using Parity = mca::hardware::EthercatPdoParityResult;
using Direction = mca::hardware::EthercatPdoDirection;

constexpr std::uint32_t mapping_word(std::uint16_t index,
                                     std::uint8_t subindex,
                                     std::uint8_t width) {
  return (static_cast<std::uint32_t>(index) << 16U) |
      (static_cast<std::uint32_t>(subindex) << 8U) | width;
}

Readback make_readback() {
  Readback result{};
  const bool first = result.add({0U, 0x2U, 0x1234U, 2U, Direction::Output,
      0x1C12U, 0U, 0x1600U, 0U, 0U, mapping_word(0x6040U, 0U, 16U)});
  const bool second = result.add({0U, 0x2U, 0x1234U, 2U, Direction::Output,
      0x1C12U, 0U, 0x1600U, 0U, 1U, mapping_word(0x607AU, 0U, 32U)});
  const bool third = result.add({0U, 0x2U, 0x1234U, 3U, Direction::Input,
      0x1C13U, 0U, 0x1A00U, 0U, 0U, mapping_word(0x6041U, 0U, 16U)});
  return first && second && third ? result : Readback{};
}

TEST(EthercatPdoReadback, MatchesExactOrderedAssignmentAndEntrySequence) {
  const auto expected = make_readback();
  const auto observed = make_readback();
  ASSERT_TRUE(expected.valid());
  ASSERT_TRUE(observed.valid());
  EXPECT_EQ(expected.compare(observed), Parity::Match);
}

TEST(EthercatPdoReadback, RejectsSlaveIdentityAndMappingWidthMismatch) {
  const auto expected = make_readback();
  auto wrong_identity = make_readback();
  auto wrong = wrong_identity[0U];
  wrong.vendor_id = 0x99U;
  Readback changed_identity{};
  ASSERT_TRUE(changed_identity.add(wrong));
  ASSERT_TRUE(changed_identity.add(wrong_identity[1U]));
  ASSERT_TRUE(changed_identity.add(wrong_identity[2U]));
  EXPECT_EQ(expected.compare(changed_identity), Parity::RecordMismatch);

  auto wrong_width = make_readback();
  Readback changed_width{};
  auto width_row = wrong_width[0U];
  width_row.raw_mapping_entry = mapping_word(0x6040U, 0U, 8U);
  ASSERT_TRUE(changed_width.add(width_row));
  ASSERT_TRUE(changed_width.add(wrong_width[1U]));
  ASSERT_TRUE(changed_width.add(wrong_width[2U]));
  EXPECT_EQ(expected.compare(changed_width), Parity::RecordMismatch);
}

TEST(EthercatPdoReadback, RejectsAssignmentAndEntryReordering) {
  const auto expected = make_readback();
  Readback reordered{};
  auto row0 = make_readback()[0U];
  auto row1 = make_readback()[1U];
  auto row2 = make_readback()[2U];
  row0.entry_position = 1U;
  row1.entry_position = 0U;
  row0.raw_mapping_entry = mapping_word(0x607AU, 0U, 32U);
  row1.raw_mapping_entry = mapping_word(0x6040U, 0U, 16U);
  ASSERT_TRUE(reordered.add(row0));
  ASSERT_TRUE(reordered.add(row1));
  ASSERT_TRUE(reordered.add(row2));
  EXPECT_EQ(expected.compare(reordered), Parity::InvalidRecord);

  Readback assignment_reordered{};
  auto tx = row2;
  tx.assignment_object = 0x1C13U;
  ASSERT_TRUE(assignment_reordered.add(tx));
  EXPECT_EQ(expected.compare(assignment_reordered), Parity::CardinalityMismatch);
}

TEST(EthercatPdoReadback, RejectsMissingAndExtraRecordsAndCapacityOverflow) {
  const auto expected = make_readback();
  Readback missing{};
  ASSERT_TRUE(missing.add(expected[0U]));
  ASSERT_TRUE(missing.add(expected[1U]));
  EXPECT_EQ(expected.compare(missing), Parity::CardinalityMismatch);

  Readback extra{};
  ASSERT_TRUE(extra.add(expected[0U]));
  ASSERT_TRUE(extra.add(expected[1U]));
  ASSERT_TRUE(extra.add(expected[2U]));
  auto duplicate = expected[2U];
  duplicate.entry_position = 1U;
  duplicate.raw_mapping_entry = mapping_word(0x606CU, 0U, 32U);
  ASSERT_TRUE(extra.add(duplicate));
  EXPECT_EQ(expected.compare(extra), Parity::CardinalityMismatch);

  Readback bounded{};
  for (std::size_t i = 0U; i < 8U; ++i)
    ASSERT_TRUE(bounded.add(expected[0U]));
  EXPECT_FALSE(bounded.add(expected[0U]));
}

TEST(EthercatPdoReadback, PreservesLegalPaddingAndRejectsMalformedOrDuplicateEntries) {
  Readback padding{};
  ASSERT_TRUE(padding.add({0U, 0x2U, 0x1234U, 2U, Direction::Output,
      0x1C12U, 0U, 0x1600U, 0U, 0U, mapping_word(0U, 0U, 8U)}));
  ASSERT_TRUE(padding.add({0U, 0x2U, 0x1234U, 2U, Direction::Output,
      0x1C12U, 0U, 0x1600U, 0U, 1U, mapping_word(0U, 0U, 8U)}));
  EXPECT_TRUE(padding.valid());

  Readback malformed{};
  ASSERT_TRUE(malformed.add({0U, 0x2U, 0x1234U, 2U, Direction::Output,
      0x1C12U, 0U, 0x1600U, 0U, 0U, mapping_word(0U, 1U, 8U)}));
  EXPECT_FALSE(malformed.valid());

  Readback duplicate{};
  const auto valid = make_readback();
  auto duplicate_a = valid[0U];
  auto duplicate_b = valid[1U];
  duplicate_b.raw_mapping_entry = duplicate_a.raw_mapping_entry;
  ASSERT_TRUE(duplicate.add(duplicate_a));
  ASSERT_TRUE(duplicate.add(duplicate_b));
  EXPECT_FALSE(duplicate.valid());
}

TEST(EthercatPdoReadback, RejectsDirectionAssignmentMismatchAndZeroWidth) {
  Readback wrong_direction{};
  ASSERT_TRUE(wrong_direction.add({0U, 0x2U, 0x1234U, 2U, Direction::Input,
      0x1C12U, 0U, 0x1600U, 0U, 0U, mapping_word(0x6040U, 0U, 16U)}));
  EXPECT_FALSE(wrong_direction.valid());

  Readback zero_width{};
  ASSERT_TRUE(zero_width.add({0U, 0x2U, 0x1234U, 2U, Direction::Output,
      0x1C12U, 0U, 0x1600U, 0U, 0U, mapping_word(0x6040U, 0U, 0U)}));
  EXPECT_FALSE(zero_width.valid());
}
}  // namespace
