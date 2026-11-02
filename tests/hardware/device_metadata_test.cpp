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


TEST(HardwareDeviceMetadata, PreservesCiA402OperationModeSemantics) {
  using namespace mca::hardware;
  AxisCapabilities axis;
  axis.supported_modes = control_mode_bit(DriveControlMode::Position) |
                         control_mode_bit(DriveControlMode::Velocity) |
                         control_mode_bit(DriveControlMode::Torque);
  axis.supported_operation_modes =
      operation_mode_bit(DriveOperationMode::ProfilePosition) |
      operation_mode_bit(DriveOperationMode::CyclicSynchronousPosition) |
      operation_mode_bit(DriveOperationMode::CyclicSynchronousVelocity);
  EXPECT_TRUE(axis.valid());
  EXPECT_TRUE(axis.supports(DriveOperationMode::ProfilePosition));
  EXPECT_TRUE(axis.supports(DriveOperationMode::CyclicSynchronousPosition));
  EXPECT_EQ(normalized_control_mode(
                DriveOperationMode::CyclicSynchronousVelocity),
            DriveControlMode::Velocity);

  axis.supported_modes = control_mode_bit(DriveControlMode::Position);
  EXPECT_FALSE(axis.valid());  // CSV cannot be advertised without velocity.
  axis.supported_operation_modes = 0x8000U;
  EXPECT_FALSE(axis.valid());
}

TEST(HardwareDeviceMetadata, ValidatesFixedCapacityDeviceIdentityAndTopology) {
  using namespace mca::hardware;
  DeviceIdentity<2> identity;
  identity.axis_count = 2;
  identity.vendor_name[0] = 'A';
  identity.product_name[0] = 'D';
  identity.serial_number[0] = '1';
  identity.firmware_version[0] = '2';
  identity.axis_names[0][0] = 'l';
  identity.axis_names[0][1] = 'e';
  identity.axis_names[0][2] = 'f';
  identity.axis_names[1][0] = 'r';
  identity.axis_names[1][1] = 'i';
  identity.axis_names[1][2] = 'g';
  identity.axis_names[1][3] = 'h';
  identity.axis_names[1][4] = 't';
  identity.vendor_id = 0x1234U;
  identity.product_code = 0x5678U;
  identity.revision = 3U;
  EXPECT_TRUE(identity.valid());

  identity.axis_names[1] = identity.axis_names[0];
  EXPECT_FALSE(identity.valid());
  identity.axis_names[1][0] = 'r';
  identity.axis_count = 3;
  EXPECT_FALSE(identity.valid());
  identity.axis_count = 1;
  identity.vendor_name.fill('x');  // Missing NUL terminator.
  EXPECT_FALSE(identity.valid());
}

TEST(HardwareDeviceMetadata, CapabilityLookupDoesNotRequireWholeDeviceValidation) {
  using namespace mca::hardware;
  DeviceCapabilities<3> capabilities;
  capabilities.axis_count = 2;
  capabilities.axes[0].supported_modes =
      control_mode_bit(DriveControlMode::Position);
  // Axis 1 is deliberately not configured. The lookup for axis 0 must remain
  // constant-time and must not scan unrelated axis metadata.
  EXPECT_TRUE(capabilities.supports(0, DriveControlMode::Position));
  EXPECT_FALSE(capabilities.supports(1, DriveControlMode::Position));
  EXPECT_FALSE(capabilities.supports(3, DriveControlMode::Position));
  EXPECT_FALSE(capabilities.valid());
}
