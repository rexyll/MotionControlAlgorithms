#include <array>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "mca/hardware/drive_commands.hpp"

namespace {

using namespace mca::hardware;

DeviceCapabilities<3> make_capabilities() {
  DeviceCapabilities<3> caps;
  caps.axis_count = 2;
  caps.axes[0].supported_modes =
      control_mode_bit(DriveControlMode::Position) |
      control_mode_bit(DriveControlMode::Velocity);
  caps.axes[0].supported_operation_modes =
      operation_mode_bit(DriveOperationMode::ProfilePosition) |
      operation_mode_bit(DriveOperationMode::CyclicSynchronousPosition) |
      operation_mode_bit(DriveOperationMode::CyclicSynchronousVelocity);
  caps.axes[1].supported_modes =
      control_mode_bit(DriveControlMode::Torque) |
      control_mode_bit(DriveControlMode::Current);
  caps.axes[1].supported_operation_modes =
      operation_mode_bit(DriveOperationMode::CyclicSynchronousTorque);
  return caps;
}

DriveCommandBatch<3> make_batch(std::uint64_t sequence) {
  DriveCommandBatch<3> batch;
  batch.count = 2;
  batch.metadata.sequence = sequence;
  batch.metadata.deadline_valid = true;
  batch.metadata.deadline_ns = 5000000U;
  batch.metadata.clock_domain = HardwareClockDomain::HostMonotonic;
  batch.setpoints[0].axis = 0;
  batch.setpoints[0].control_mode = DriveControlMode::Position;
  batch.setpoints[0].operation_mode =
      DriveOperationMode::CyclicSynchronousPosition;
  batch.setpoints[0].position_rad = 1.25;
  batch.setpoints[1].axis = 1;
  batch.setpoints[1].control_mode = DriveControlMode::Torque;
  batch.setpoints[1].operation_mode =
      DriveOperationMode::CyclicSynchronousTorque;
  batch.setpoints[1].torque_nm = 0.8;
  return batch;
}

class TransactionalFakeSink final : public IDriveCommandSink<3> {
 public:
  explicit TransactionalFakeSink(DeviceCapabilities<3> caps)
      : caps_(caps) {}

  mca::core::Status write_batch(const DriveCommandBatch<3>& batch,
                                std::uint64_t now_ns,
                                HardwareClockDomain domain) noexcept override {
    const auto status = validate_drive_command_batch(
        caps_, batch, now_ns, domain, last_sequence_);
    if (!status.ok()) return status;
    accepted_ = batch;
    last_sequence_ = batch.metadata.sequence;
    ++writes_;
    return {};
  }

  std::uint64_t last_accepted_sequence() const noexcept override {
    return last_sequence_;
  }
  std::size_t writes() const noexcept { return writes_; }
  const DriveCommandBatch<3>& accepted() const noexcept { return accepted_; }

 private:
  DeviceCapabilities<3> caps_;
  DriveCommandBatch<3> accepted_{};
  std::uint64_t last_sequence_{0};
  std::size_t writes_{0};
};

TEST(DriveCommandBatch, AcceptsSupportedProfileAndCyclicModes) {
  const auto caps = make_capabilities();
  auto batch = make_batch(1);
  EXPECT_TRUE(batch.valid_shape());
  EXPECT_TRUE(validate_drive_command_batch(
                  caps, batch, 100U, HardwareClockDomain::HostMonotonic, 0U)
                  .ok());

  batch.setpoints[0].operation_mode = DriveOperationMode::ProfilePosition;
  EXPECT_TRUE(validate_drive_command_batch(
                  caps, batch, 100U, HardwareClockDomain::HostMonotonic, 0U)
                  .ok());
}

TEST(DriveCommandBatch, RejectsUnsupportedModesAxesAndDuplicateAxes) {
  const auto caps = make_capabilities();
  auto batch = make_batch(1);
  batch.setpoints[0].operation_mode =
      DriveOperationMode::CyclicSynchronousVelocity;
  batch.setpoints[0].control_mode = DriveControlMode::Velocity;
  batch.setpoints[0].velocity_rad_s = 2.0;
  EXPECT_TRUE(validate_drive_command_batch(
                  caps, batch, 100U, HardwareClockDomain::HostMonotonic, 0U)
                  .ok());

  batch.setpoints[0].operation_mode = DriveOperationMode::ProfileVelocity;
  EXPECT_EQ(validate_drive_command_batch(
                caps, batch, 100U, HardwareClockDomain::HostMonotonic, 0U)
                .code,
            mca::core::StatusCode::ConstraintViolation);
  batch = make_batch(1);
  batch.setpoints[1].axis = 0;
  EXPECT_FALSE(batch.valid_shape());
  batch = make_batch(1);
  batch.setpoints[1].axis = 2;
  EXPECT_EQ(validate_drive_command_batch(
                caps, batch, 100U, HardwareClockDomain::HostMonotonic, 0U)
                .code,
            mca::core::StatusCode::ConstraintViolation);
}

TEST(DriveCommandBatch, RejectsStaleExpiredAndClockMismatchedCommands) {
  const auto caps = make_capabilities();
  auto batch = make_batch(5);
  EXPECT_EQ(validate_drive_command_batch(
                caps, batch, 100U, HardwareClockDomain::HostMonotonic, 5U)
                .code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_EQ(validate_drive_command_batch(
                caps, batch, 5000000U, HardwareClockDomain::HostMonotonic, 0U)
                .code,
            mca::core::StatusCode::RuntimeError);
  EXPECT_EQ(validate_drive_command_batch(
                caps, batch, 100U, HardwareClockDomain::DeviceMonotonic, 0U)
                .code,
            mca::core::StatusCode::RuntimeError);
}

TEST(DriveCommandBatch, RejectsNonFiniteActiveSetpointAndMalformedBatch) {
  auto batch = make_batch(1);
  batch.setpoints[0].position_rad =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(batch.valid_shape());
  batch = make_batch(1);
  batch.count = 4;
  EXPECT_FALSE(batch.valid_shape());
  batch = make_batch(1);
  batch.metadata.sequence = 0;
  EXPECT_FALSE(batch.valid_shape());
}

TEST(DriveCommandBatch, KeepsVendorCurrentControlSeparateFromCiA402Modes) {
  const auto caps = make_capabilities();
  auto batch = make_batch(1);
  batch.count = 1;
  batch.setpoints[0].axis = 1;
  batch.setpoints[0].control_mode = DriveControlMode::Current;
  batch.setpoints[0].current_a = 2.5;
  EXPECT_TRUE(validate_drive_command_batch(
                  caps, batch, 100U, HardwareClockDomain::HostMonotonic, 0U)
                  .ok());

  auto no_current = caps;
  no_current.axes[1].supported_modes =
      control_mode_bit(DriveControlMode::Torque);
  EXPECT_EQ(validate_drive_command_batch(
                no_current, batch, 100U, HardwareClockDomain::HostMonotonic,
                0U).code,
            mca::core::StatusCode::ConstraintViolation);
}

TEST(DriveCommandBatch, FailedBatchDoesNotPartiallyCommitOrAdvanceSequence) {
  TransactionalFakeSink sink(make_capabilities());
  auto good = make_batch(1);
  ASSERT_TRUE(sink.write_batch(good, 100U,
                               HardwareClockDomain::HostMonotonic).ok());
  ASSERT_EQ(sink.writes(), 1U);
  ASSERT_EQ(sink.last_accepted_sequence(), 1U);

  auto invalid = make_batch(2);
  invalid.setpoints[1].operation_mode = DriveOperationMode::ProfileTorque;
  EXPECT_EQ(sink.write_batch(invalid, 100U,
                             HardwareClockDomain::HostMonotonic).code,
            mca::core::StatusCode::ConstraintViolation);
  EXPECT_EQ(sink.writes(), 1U);
  EXPECT_EQ(sink.last_accepted_sequence(), 1U);
  EXPECT_DOUBLE_EQ(sink.accepted().setpoints[0].position_rad, 1.25);
}

}  // namespace
