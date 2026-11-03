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

Request read_request(std::uint64_t id, std::uint64_t deadline,
                     std::size_t max_response_size = 4U) {
  Request r{};
  r.request_id = id;
  r.slave_position = 5U;
  r.index = 0x2000U;
  r.subindex = 1U;
  r.operation = mca::hardware::EthercatSdoOperation::Read;
  r.max_response_size = max_response_size;
  r.deadline_timestamp_ns = deadline;
  return r;
}
}  // namespace

TEST(EthercatIghFakeApi, CollectsMasterAndSlaveRuntimeHealth) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));

  ASSERT_TRUE(adapter.exchange().ok());
  const auto& health = adapter.runtime_health();
  ASSERT_TRUE(health.valid());
  EXPECT_TRUE(health.master_state_valid);
  EXPECT_TRUE(health.link_up);
  EXPECT_EQ(health.slaves_responding, 1U);
  EXPECT_EQ(health.al_states, EC_AL_STATE_OP);
  ASSERT_EQ(health.slave_count, 1U);
  EXPECT_EQ(health.slaves[0].station_position, 5U);
  EXPECT_TRUE(health.slaves[0].configured);
  EXPECT_TRUE(health.slaves[0].state_valid);
  EXPECT_TRUE(health.slaves[0].online);
  EXPECT_TRUE(health.slaves[0].operational);
  EXPECT_EQ(health.slaves[0].al_state, EC_AL_STATE_OP);
  EXPECT_EQ(health.sequence, adapter.cycle_sequence());
}

TEST(EthercatIghFakeApi, ReportsMasterLinkLossIndependentlyFromCompleteDomainWkc) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  fake_igh::set_master_state(false, 0U, EC_AL_STATE_SAFEOP);

  ASSERT_TRUE(adapter.exchange().ok());
  const auto& health = adapter.runtime_health();
  ASSERT_TRUE(health.valid());
  EXPECT_FALSE(health.link_up);
  EXPECT_EQ(health.slaves_responding, 0U);
  EXPECT_EQ(adapter.domain_working_counter_complete(7U), true);
}

TEST(EthercatIghFakeApi, ReportsSlaveOfflineAndSafeOperationalIndependentlyFromMaster) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  fake_igh::set_slave_state(false, false, EC_AL_STATE_SAFEOP);

  ASSERT_TRUE(adapter.exchange().ok());
  const auto& health = adapter.runtime_health();
  ASSERT_TRUE(health.valid());
  EXPECT_TRUE(health.link_up);
  ASSERT_EQ(health.slave_count, 1U);
  EXPECT_FALSE(health.slaves[0].online);
  EXPECT_FALSE(health.slaves[0].operational);
  EXPECT_EQ(health.slaves[0].al_state, EC_AL_STATE_SAFEOP);
  EXPECT_TRUE(adapter.domain_working_counter_complete(7U));
}

TEST(EthercatIghFakeApi, SlaveDiagnosticReadFailureKeepsCyclicQueueAndSendMoving) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  fake_igh::fail_slave_state(true);

  EXPECT_FALSE(adapter.exchange().ok());
  const auto& health = adapter.runtime_health();
  EXPECT_TRUE(health.master_state_valid);
  ASSERT_EQ(health.slave_count, 1U);
  EXPECT_TRUE(health.slaves[0].configured);
  EXPECT_FALSE(health.slaves[0].state_valid);
  EXPECT_EQ(health.slaves[0].state_error, -1);
  EXPECT_EQ(fake_igh::queue_calls(), 1U);
  EXPECT_EQ(fake_igh::send_calls(), 1U);
}

TEST(EthercatIghFakeApi, DiagnosticReadFailureKeepsCyclicQueueAndSendMoving) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  fake_igh::fail_master_state(true);

  EXPECT_FALSE(adapter.exchange().ok());
  const auto& health = adapter.runtime_health();
  EXPECT_FALSE(health.master_state_valid);
  EXPECT_EQ(health.master_state_error, -1);
  EXPECT_EQ(fake_igh::queue_calls(), 1U);
  EXPECT_EQ(fake_igh::send_calls(), 1U);
}

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

TEST(EthercatIghFakeApi, ReadsFromRelocatedBufferOnlyAfterTerminalSuccess) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  ASSERT_TRUE(adapter.enqueue_sdo(read_request(2U, 500U)).ok());
  ASSERT_NE(fake_igh::request(0U), nullptr);

  auto* busy_buffer = ecrt_sdo_request_data(fake_igh::request(0U));
  ASSERT_NE(busy_buffer, nullptr);
  busy_buffer[0] = 0xAAU;

  Result result{};
  ASSERT_TRUE(adapter.poll_sdo(2U, 100U, result).ok());
  EXPECT_EQ(result.state, mca::hardware::EthercatMailboxResultState::Pending);

  const std::array<std::uint8_t, 3U> bytes{{0x21U, 0x43U, 0x65U}};
  fake_igh::relocate_and_complete(0U, bytes.data(), bytes.size());
  auto* terminal_buffer = ecrt_sdo_request_data(fake_igh::request(0U));
  ASSERT_NE(terminal_buffer, busy_buffer);

  ASSERT_TRUE(adapter.poll_sdo(2U, 101U, result).ok());
  EXPECT_EQ(result.state, mca::hardware::EthercatMailboxResultState::Completed);
  ASSERT_EQ(result.payload_size, bytes.size());
  EXPECT_EQ(result.payload[0], 0x21U);
  EXPECT_EQ(result.payload[1], 0x43U);
  EXPECT_EQ(result.payload[2], 0x65U);
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

TEST(EthercatIghFakeApi, TimedOutBusySlotIsReleasedAfterTerminalError) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  ASSERT_TRUE(adapter.enqueue_sdo(read_request(23U, 100U)).ok());

  Result result{};
  ASSERT_TRUE(adapter.poll_sdo(23U, 100U, result).ok());
  EXPECT_EQ(result.state, mca::hardware::EthercatMailboxResultState::TimedOut);

  fake_igh::fail(0U);
  ASSERT_TRUE(adapter.poll_sdo(23U, 101U, result).ok());
  EXPECT_EQ(result.state, mca::hardware::EthercatMailboxResultState::TimedOut);
  ASSERT_TRUE(adapter.enqueue_sdo(read_request(24U, 300U)).ok());
  EXPECT_EQ(fake_igh::read_calls(0U), 2U);
}

TEST(EthercatIghFakeApi, MapsIgHErrorToFailedWithoutInventingAbortCodeAndReusesSlot) {
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
  ASSERT_TRUE(adapter.enqueue_sdo(read_request(31U, 500U)).ok());
  EXPECT_EQ(fake_igh::read_calls(0U), 2U);
}

TEST(EthercatIghFakeApi, RejectsReadResponseAboveAdapterCapacity) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  EXPECT_FALSE(adapter.enqueue_sdo(read_request(32U, 500U, 257U)).ok());
  EXPECT_EQ(fake_igh::read_calls(0U), 0U);
  EXPECT_EQ(fake_igh::read_calls(1U), 0U);
}

TEST(EthercatIghFakeApi, RejectsSuccessfulReadLargerThanCallerBound) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  ASSERT_NE(fake_igh::request(0U), nullptr);
  EXPECT_EQ(ecrt_sdo_request_data_size(fake_igh::request(0U)), 4U);

  ASSERT_TRUE(adapter.enqueue_sdo(read_request(33U, 500U, 3U)).ok());
  const std::array<std::uint8_t, 4U> bytes{{0x10U, 0x20U, 0x30U, 0x40U}};
  fake_igh::complete(0U, bytes.data(), bytes.size());

  Result result{};
  ASSERT_TRUE(adapter.poll_sdo(33U, 100U, result).ok());
  EXPECT_EQ(result.state, mca::hardware::EthercatMailboxResultState::Failed);
  EXPECT_EQ(result.payload_size, 0U);
  EXPECT_EQ(result.payload[0], 0U);
  EXPECT_TRUE(adapter.enqueue_sdo(read_request(34U, 500U, 4U)).ok());
}

TEST(EthercatIghFakeApi, AcceptsWriteAtExactTransferCapacityAndRejectsOverflow) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));

  Request exact{};
  exact.request_id = 40U;
  exact.slave_position = 5U;
  exact.index = 0x2000U;
  exact.subindex = 1U;
  exact.operation = mca::hardware::EthercatSdoOperation::Write;
  exact.payload = {{1U, 2U, 3U, 4U}};
  exact.payload_size = 4U;
  exact.deadline_timestamp_ns = 500U;
  ASSERT_TRUE(adapter.enqueue_sdo(exact).ok());
  EXPECT_EQ(fake_igh::write_calls(0U), 1U);

  Request overflow = exact;
  overflow.request_id = 41U;
  overflow.payload_size = 5U;
  EXPECT_FALSE(adapter.enqueue_sdo(overflow).ok());
  EXPECT_EQ(fake_igh::write_calls(1U), 0U);
}

TEST(EthercatIghFakeApi, CopiesFixedSizeWritePayloadBeforeSubmission) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  Request request{};
  request.request_id = 50U;
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

namespace {
Adapter::SdoProfile two_slot_profile() {
  Adapter::SdoProfile profile{};
  profile.slot_count = 2U;
  profile.slots[0] = {5U, 0x2000U, 1U, 4U, 50U};
  profile.slots[1] = profile.slots[0];
  return profile;
}

bool configure_without_sdo(Adapter& adapter) {
  if (!adapter.request_master().ok() || !adapter.discover().ok()) return false;
  std::array<Adapter::PdoBinding, 128U> bindings{};
  bindings[0] = {5U, 0x2U, 0x1234U, 0x6041U, 0U, 16U, 7U,
                 mca::hardware::EthercatPdoDirection::Input};
  return adapter.configure(bindings, 1U).ok();
}
}  // namespace

TEST(EthercatIghFakeApi, RollsBackAllRequestsWhenSecondSdoRequestCreationFails) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(configure_without_sdo(adapter));
  fake_igh::fail_create_at(1U);

  EXPECT_FALSE(adapter.configure_sdo_profile(two_slot_profile()).ok());
  EXPECT_EQ(fake_igh::release_calls(), 1U);
  EXPECT_EQ(fake_igh::live_request_count(), 0U);
  EXPECT_EQ(adapter.state().state(), mca::hardware::EthercatMasterState::Offline);
  EXPECT_TRUE(adapter.shutdown().ok());
  EXPECT_EQ(fake_igh::release_calls(), 1U);
}

TEST(EthercatIghFakeApi, RollsBackCreatedRequestsWhenTimeoutConfigurationFails) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(configure_without_sdo(adapter));
  fake_igh::fail_timeout_at(1U);

  EXPECT_FALSE(adapter.configure_sdo_profile(two_slot_profile()).ok());
  EXPECT_EQ(fake_igh::release_calls(), 1U);
  EXPECT_EQ(fake_igh::live_request_count(), 0U);
  EXPECT_EQ(adapter.state().state(), mca::hardware::EthercatMasterState::Offline);
  EXPECT_TRUE(adapter.shutdown().ok());
  EXPECT_EQ(fake_igh::release_calls(), 1U);
}


TEST(EthercatIghFakeApi, ActivationFailureDiscardsConfiguredMaster) {
  fake_igh::reset();
  Adapter::Config config{}; config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(configure_without_sdo(adapter));
  fake_igh::fail_activate(true);
  EXPECT_FALSE(adapter.activate().ok());
  EXPECT_EQ(adapter.state().state(), mca::hardware::EthercatMasterState::Offline);
  EXPECT_EQ(fake_igh::release_calls(), 1U);
  fake_igh::fail_activate(false);
  EXPECT_FALSE(adapter.activate().ok());
  EXPECT_EQ(fake_igh::activate_calls(), 1U);
  EXPECT_TRUE(adapter.shutdown().ok());
  EXPECT_EQ(fake_igh::release_calls(), 1U);
}

TEST(EthercatIghFakeApi, NullDomainDataDiscardsMaster) {
  fake_igh::reset();
  Adapter::Config config{}; config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(configure_without_sdo(adapter));
  fake_igh::fail_domain_data(true);
  EXPECT_FALSE(adapter.activate().ok());
  EXPECT_EQ(fake_igh::deactivate_calls(), 1U);
  EXPECT_EQ(adapter.state().state(), mca::hardware::EthercatMasterState::Offline);
  EXPECT_EQ(fake_igh::release_calls(), 1U);
  fake_igh::fail_domain_data(false);
  EXPECT_FALSE(adapter.activate().ok());
  EXPECT_TRUE(adapter.shutdown().ok());
  EXPECT_EQ(fake_igh::release_calls(), 1U);
}

TEST(EthercatIghFakeApi, ZeroDomainSizeDiscardsMaster) {
  fake_igh::reset();
  Adapter::Config config{}; config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(configure_without_sdo(adapter));
  fake_igh::zero_domain_size(true);
  EXPECT_FALSE(adapter.activate().ok());
  EXPECT_EQ(fake_igh::deactivate_calls(), 1U);
  EXPECT_EQ(adapter.state().state(), mca::hardware::EthercatMasterState::Offline);
  EXPECT_EQ(fake_igh::release_calls(), 1U);
  fake_igh::zero_domain_size(false);
  EXPECT_FALSE(adapter.activate().ok());
  EXPECT_TRUE(adapter.shutdown().ok());
  EXPECT_EQ(fake_igh::release_calls(), 1U);
}

TEST(EthercatIghFakeApi, FailedActivationRollbackReleasesUncertainMaster) {
  fake_igh::reset();
  Adapter::Config config{}; config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(configure_without_sdo(adapter));
  fake_igh::fail_domain_data(true);
  fake_igh::fail_deactivate(true);
  EXPECT_FALSE(adapter.activate().ok());
  EXPECT_EQ(fake_igh::deactivate_calls(), 1U);
  EXPECT_EQ(fake_igh::release_calls(), 1U);
  EXPECT_EQ(adapter.state().state(), mca::hardware::EthercatMasterState::Offline);
  EXPECT_FALSE(adapter.activate().ok());
}

TEST(EthercatIghFakeApi, FailedShutdownDeactivationStillReleasesMasterAndReportsError) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  fake_igh::fail_deactivate(true);

  EXPECT_FALSE(adapter.shutdown().ok());
  EXPECT_EQ(fake_igh::deactivate_calls(), 1U);
  EXPECT_EQ(fake_igh::release_calls(), 1U);
  EXPECT_EQ(adapter.state().state(), mca::hardware::EthercatMasterState::Offline);
  EXPECT_TRUE(adapter.shutdown().ok());
  EXPECT_EQ(fake_igh::deactivate_calls(), 1U);
  EXPECT_EQ(fake_igh::release_calls(), 1U);
}

TEST(EthercatIghFakeApi, ShutdownDeactivatesAndReleasesMasterExactlyOnce) {
  fake_igh::reset();
  Adapter::Config config{};
  config.cycle_time_ns = 1'000'000U;
  Adapter adapter(config);
  ASSERT_TRUE(activate_with_two_slots(adapter));
  ASSERT_TRUE(adapter.enqueue_sdo(read_request(60U, 500U)).ok());

  EXPECT_TRUE(adapter.shutdown().ok());
  EXPECT_EQ(fake_igh::deactivate_calls(), 1U);
  EXPECT_EQ(fake_igh::release_calls(), 1U);
  EXPECT_EQ(fake_igh::live_request_count(), 0U);
  EXPECT_TRUE(adapter.shutdown().ok());
  EXPECT_EQ(fake_igh::deactivate_calls(), 1U);
  EXPECT_EQ(fake_igh::release_calls(), 1U);
}
