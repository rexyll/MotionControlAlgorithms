#include <array>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "mca/hardware/cia402_pdo_codec.hpp"

namespace {
using namespace mca::hardware;

TEST(Cia402PdoCodec, RoundTripsAllDriveObjectScalarWidths) {
  std::array<std::uint8_t, 16U> image{};
  ASSERT_TRUE(cia402_write_u16(image.data(), image.size(), 0U, 0x6040U));
  ASSERT_TRUE(cia402_write_i32(image.data(), image.size(), 16U,
                               std::numeric_limits<std::int32_t>::min()));
  ASSERT_TRUE(cia402_write_i32(image.data(), image.size(), 48U,
                               std::numeric_limits<std::int32_t>::max()));
  ASSERT_TRUE(cia402_write_i8(image.data(), image.size(), 80U,
                              std::numeric_limits<std::int8_t>::min()));
  ASSERT_TRUE(cia402_write_u8(image.data(), image.size(), 88U, 0x08U));

  std::uint16_t controlword = 0U;
  std::int32_t minimum = 0;
  std::int32_t maximum = 0;
  std::int8_t mode = 0;
  std::uint8_t mode_display = 0U;
  EXPECT_TRUE(cia402_read_u16(image.data(), image.size(), 0U, controlword));
  EXPECT_TRUE(cia402_read_i32(image.data(), image.size(), 16U, minimum));
  EXPECT_TRUE(cia402_read_i32(image.data(), image.size(), 48U, maximum));
  EXPECT_TRUE(cia402_read_i8(image.data(), image.size(), 80U, mode));
  EXPECT_TRUE(cia402_read_u8(image.data(), image.size(), 88U, mode_display));
  EXPECT_EQ(controlword, 0x6040U);
  EXPECT_EQ(minimum, std::numeric_limits<std::int32_t>::min());
  EXPECT_EQ(maximum, std::numeric_limits<std::int32_t>::max());
  EXPECT_EQ(mode, std::numeric_limits<std::int8_t>::min());
  EXPECT_EQ(mode_display, 0x08U);
}

TEST(Cia402PdoCodec, RejectsOutOfRangeWithoutModifyingImage) {
  std::array<std::uint8_t, 4U> image{0xA5U, 0x5AU, 0xC3U, 0x3CU};
  const auto before = image;
  EXPECT_FALSE(cia402_write_u16(image.data(), image.size(), 24U, 0xFFFFU));
  EXPECT_FALSE(cia402_write_i32(image.data(), image.size(), 8U, -1));
  std::int32_t untouched = 123;
  EXPECT_FALSE(cia402_read_i32(nullptr, image.size(), 0U, untouched));
  EXPECT_EQ(untouched, 123);
  EXPECT_EQ(image, before);
}

TEST(Cia402PdoCodec, HandlesNegativeModeAndPositionValues) {
  std::array<std::uint8_t, 5U> image{};
  ASSERT_TRUE(cia402_write_i8(image.data(), image.size(), 0U, -1));
  ASSERT_TRUE(cia402_write_i32(image.data(), image.size(), 8U, -123456789));
  std::int8_t mode = 0;
  std::int32_t position = 0;
  EXPECT_TRUE(cia402_read_i8(image.data(), image.size(), 0U, mode));
  EXPECT_TRUE(cia402_read_i32(image.data(), image.size(), 8U, position));
  EXPECT_EQ(mode, -1);
  EXPECT_EQ(position, -123456789);
}
}  // namespace
