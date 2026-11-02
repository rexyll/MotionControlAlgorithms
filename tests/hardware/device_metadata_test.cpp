#include <cstdint>

#include <gtest/gtest.h>

#include "mca/hardware/device_metadata.hpp"

TEST(HardwareDeviceMetadata, ReportsPerAxisControlModes) {
  mca::hardware::DeviceCapabilities<4> capabilities;
  capabilities.axis_count = 2;
  capabilities.axes[0].supported_modes =
      mca::hardware::control_mode_bit(
          mca::hardware::DriveControlMode::Position) |
      mca::hardware::control_mode_bit(
          mca::hardware::DriveControlMode::Torque);
  capabilities.axes[0].supports_cyclic_modes = true;
  capabilities.axes[1].supported_modes =
      mca::hardware::control_mode_bit(
          mca::hardware::DriveControlMode::Velocity);
  capabilities.coherent_multi_axis_state = true;

  ASSERT_TRUE(capabilities.valid());
  EXPECT_TRUE(capabilities.supports(
      0, mca::hardware::DriveControlMode::Position));
  EXPECT_TRUE(capabilities.supports(
      0, mca::hardware::DriveControlMode::Torque));
  EXPECT_FALSE(capabilities.supports(
      0, mca::hardware::DriveControlMode::Velocity));
  EXPECT_TRUE(capabilities.supports(
      1, mca::hardware::DriveControlMode::Velocity));
  EXPECT_FALSE(capabilities.supports(
      2, mca::hardware::DriveControlMode::Velocity));
}

TEST(HardwareDeviceMetadata, RejectsInvalidAxisCountsAndModeMasks) {
  mca::hardware::DeviceCapabilities<2> capabilities;
  EXPECT_FALSE(capabilities.valid());

  capabilities.axis_count = 3;
  EXPECT_FALSE(capabilities.valid());

  capabilities.axis_count = 1;
  capabilities.axes[0].supported_modes = 0x80U;
  EXPECT_FALSE(capabilities.valid());

  capabilities.axes[0].supported_modes =
      mca::hardware::control_mode_bit(
          mca::hardware::DriveControlMode::Current);
  EXPECT_TRUE(capabilities.valid());
}

TEST(HardwareDeviceMetadata, ValidatesSampleTimestampAndSequence) {
  mca::hardware::HardwareSampleMetadata sample;
  EXPECT_FALSE(sample.valid());

  sample.sequence = 1;
  EXPECT_TRUE(sample.valid());

  sample.timestamp_valid = true;
  EXPECT_FALSE(sample.valid());

  sample.timestamp_ns = 123456U;
  sample.clock_domain = mca::hardware::HardwareClockDomain::HostMonotonic;
  sample.coherent = true;
  EXPECT_TRUE(sample.valid());

  sample.clock_domain = mca::hardware::HardwareClockDomain::Unknown;
  EXPECT_FALSE(sample.valid());
}

TEST(HardwareDeviceMetadata, EnforcesCommandDeadlineAndClockDomain) {
  mca::hardware::HardwareCommandMetadata command;
  command.sequence = 7;
  command.deadline_valid = true;
  command.deadline_ns = 1000;
  command.clock_domain = mca::hardware::HardwareClockDomain::HostMonotonic;

  ASSERT_TRUE(command.valid());
  EXPECT_TRUE(command.deadline_compatible(
      mca::hardware::HardwareClockDomain::HostMonotonic));
  EXPECT_FALSE(command.deadline_compatible(
      mca::hardware::HardwareClockDomain::EthercatDistributed));
  EXPECT_FALSE(command.expired(
      999, mca::hardware::HardwareClockDomain::HostMonotonic));
  EXPECT_TRUE(command.expired(
      1000, mca::hardware::HardwareClockDomain::HostMonotonic));
  EXPECT_TRUE(command.expired(
      1, mca::hardware::HardwareClockDomain::EthercatDistributed));

  command.deadline_valid = false;
  EXPECT_TRUE(command.valid());
  EXPECT_FALSE(command.expired(
      UINT64_MAX, mca::hardware::HardwareClockDomain::Unknown));
}
