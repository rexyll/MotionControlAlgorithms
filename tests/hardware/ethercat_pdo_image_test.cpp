#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "mca/hardware/ethercat_pdo_image.hpp"

namespace {

TEST(EthercatPdoImage, ReadsAndWritesUnalignedFieldsWithoutTouchingNeighbors) {
  std::array<std::uint8_t, 3> image{0xA5U, 0x5AU, 0xC3U};

  std::uint64_t value = 0U;
  ASSERT_TRUE(mca::hardware::ethercat_read_pdo_bits(image.data(), image.size(), 3U, 11U, value));
  EXPECT_EQ(value, 0x14DU);

  ASSERT_TRUE(mca::hardware::ethercat_write_pdo_bits(image.data(), image.size(), 3U, 11U, 0x2A5U));

  std::uint64_t round_trip = 0U;
  ASSERT_TRUE(mca::hardware::ethercat_read_pdo_bits(image.data(), image.size(), 3U, 11U, round_trip));
  EXPECT_EQ(round_trip, 0x2A5U);
  EXPECT_EQ(image[2] & 0xC0U, 0xC0U);
  EXPECT_EQ(image[0] & 0x07U, 0x05U);
}

TEST(EthercatPdoImage, SupportsExactly64Bits) {
  std::array<std::uint8_t, 8> image{};
  constexpr std::uint64_t kValue = 0xFEDCBA9876543210ULL;

  ASSERT_TRUE(mca::hardware::ethercat_write_pdo_bits(image.data(), image.size(), 0U, 64U, kValue));

  std::uint64_t value = 0U;
  ASSERT_TRUE(mca::hardware::ethercat_read_pdo_bits(image.data(), image.size(), 0U, 64U, value));
  EXPECT_EQ(value, kValue);
}

TEST(EthercatPdoImage, RejectsInvalidRangesAndPreservesOutputOnFailedRead) {
  std::array<std::uint8_t, 2> image{0x12U, 0x34U};
  std::uint64_t value = 0xDEADBEEFCAFEBABEULL;

  EXPECT_FALSE(mca::hardware::ethercat_read_pdo_bits(image.data(), image.size(), 8U, 9U, value));
  EXPECT_EQ(value, 0xDEADBEEFCAFEBABEULL);
  EXPECT_FALSE(mca::hardware::ethercat_write_pdo_bits(image.data(), image.size(), 8U, 9U, 0x1FFU));
  EXPECT_EQ(image[0], 0x12U);
  EXPECT_EQ(image[1], 0x34U);
  EXPECT_FALSE(mca::hardware::ethercat_read_pdo_bits(image.data(), image.size(), 0U, 0U, value));
  EXPECT_FALSE(mca::hardware::ethercat_write_pdo_bits(image.data(), image.size(), 0U, 65U, 0U));
}

}  // namespace
