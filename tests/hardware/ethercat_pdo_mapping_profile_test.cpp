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
