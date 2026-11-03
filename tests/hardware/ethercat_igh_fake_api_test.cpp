#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "fake_igh_api.hpp"
#include "mca/hardware/ethercat_igh_adapter.hpp"

namespace {
using Adapter = mca::hardware::EthercatIghAdapter<16U, 4U, 128U>;
using Request = Adapter::SdoRequest;
using Result = Adapter::SdoResult;

bool activate_with_two_slots(Adapter& adapter) {
  if (!adapter.request_master().ok() || !adapter.discover().ok()) return false;
  std::array<Adapter::PdoBinding, 128U> bindings{};
  bindings[0] = {5U, 0x2U, 0x1234U, 0x6041U, 0U, 16U, 7U,
                 mca::hardware::EthercatPdoDirection::Input};
  if (!adapter.configure(bindings, 1U).ok()) return false;

  Adapter::SdoProfile profile{};
  profile.slot_count = 2U;
  profile.slots[0] = {5U, 0x2000U, 1U, 4U, 50U};
  profile.slots[1] = profile.slots[0];
  if (!adapter.configure_sdo_profile(profile).ok()) return false;
  return adapter.activate().ok();
}

Request read_request(std::uint64_t id, std::uint64_t deadline) {
  Request r{};
  r.request_id = id;
  r.slave_position = 5U;
  r.index = 0x2000U;
  r.subindex = 1U;
  r.operation = mca::hardware::EthercatSdoOperation::Read;
  r.max_response_size = 4U;
  r.deadline_timestamp_ns = deadline;
  return r;
}
}  // namespace

TEST(EthercatIghFakeApi, ReadsOnlyAfterSuccessAndCopiesTerminalBuffer) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  ASSERT_EQ(fake_igh::request_count(), 2U);

  const auto request = read_request(1U, 500U);
  ASSERT_TRUE(adapter.enqueue_sdo(request).ok());
  EXPECT_EQ(fake_igh::read_calls(0U), 1U);
  EXPECT_EQ(fake_igh::read_calls(1U), 0U);

  Result result{};
  ASSERT_TRUE(adapter.poll_sdo(1U, 100U, result).ok());
  EXPECT_EQ(result.state, mca::hardware::EthercatMailboxResultState::Pending);

  const std::array<std::uint8_t, 3U> bytes{{0x12U, 0x34U, 0x56U}};
  fake_igh::complete(0U, bytes.data(), bytes.size());
  ASSERT_TRUE(adapter.poll_sdo(1U, 101U, result).ok());
  EXPECT_EQ(result.state, mca::hardware::EthercatMailboxResultState::Completed);
  ASSERT_EQ(result.payload_size, bytes.size());
  EXPECT_EQ(result.payload[0], 0x12U);
  EXPECT_EQ(result.payload[1], 0x34U);
  EXPECT_EQ(result.payload[2], 0x56U);
}

TEST(EthercatIghFakeApi, SupportsIndependentConcurrentSlotsAndRejectsDuplicateIds) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  ASSERT_TRUE(adapter.enqueue_sdo(read_request(10U, 500U)).ok());
  EXPECT_FALSE(adapter.enqueue_sdo(read_request(10U, 600U)).ok());
  ASSERT_TRUE(adapter.enqueue_sdo(read_request(11U, 600U)).ok());
  EXPECT_EQ(fake_igh::read_calls(0U), 1U);
  EXPECT_EQ(fake_igh::read_calls(1U), 1U);
}

TEST(EthercatIghFakeApi, SkipsUnownedBackendSlotThatIsAlreadyBusy) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  fake_igh::set_busy(0U);
  ASSERT_TRUE(adapter.enqueue_sdo(read_request(15U, 500U)).ok());
  EXPECT_EQ(fake_igh::read_calls(0U), 0U);
  EXPECT_EQ(fake_igh::read_calls(1U), 1U);
}

TEST(EthercatIghFakeApi, QuarantinesTimedOutBusySlotUntilTerminalObservation) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  ASSERT_TRUE(adapter.enqueue_sdo(read_request(20U, 100U)).ok());

  Result result{};
  ASSERT_TRUE(adapter.poll_sdo(20U, 100U, result).ok());
  EXPECT_EQ(result.state, mca::hardware::EthercatMailboxResultState::TimedOut);

  ASSERT_TRUE(adapter.enqueue_sdo(read_request(21U, 200U)).ok());
  EXPECT_FALSE(adapter.enqueue_sdo(read_request(22U, 300U)).ok());

  fake_igh::complete(0U, nullptr, 0U);
  ASSERT_TRUE(adapter.poll_sdo(20U, 101U, result).ok());
  EXPECT_EQ(result.state, mca::hardware::EthercatMailboxResultState::TimedOut);
  EXPECT_TRUE(adapter.enqueue_sdo(read_request(22U, 300U)).ok());
}

TEST(EthercatIghFakeApi, MapsIgHErrorToFailedWithoutInventingAbortCode) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  ASSERT_TRUE(adapter.enqueue_sdo(read_request(30U, 500U)).ok());
  fake_igh::fail(0U);

  Result result{};
  ASSERT_TRUE(adapter.poll_sdo(30U, 100U, result).ok());
  EXPECT_EQ(result.state, mca::hardware::EthercatMailboxResultState::Failed);
  EXPECT_EQ(result.abort_code, 0U);
}

TEST(EthercatIghFakeApi, CopiesFixedSizeWritePayloadBeforeSubmission) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  Request request{};
  request.request_id = 40U;
  request.slave_position = 5U;
  request.index = 0x2000U;
  request.subindex = 1U;
  request.operation = mca::hardware::EthercatSdoOperation::Write;
  request.payload = {{1U, 2U, 3U, 4U}};
  request.payload_size = 4U;
  request.deadline_timestamp_ns = 500U;
  ASSERT_TRUE(adapter.enqueue_sdo(request).ok());
  EXPECT_EQ(fake_igh::write_calls(0U), 1U);
  ASSERT_NE(fake_igh::request(0U), nullptr);
  EXPECT_EQ(ecrt_sdo_request_data(fake_igh::request(0U))[0], 1U);
  EXPECT_EQ(ecrt_sdo_request_data(fake_igh::request(0U))[3], 4U);
}
