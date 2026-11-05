#include <cstddef>
#include <cstdint>

#include <gtest/gtest.h>

#include "mca/hardware/ethercat_master.hpp"

namespace {

using namespace mca::hardware;

using TestMaster = IEthercatMaster<4U, 2U, 8U, 16U>;

class FakeEthercatMaster final : public TestMaster {
 public:
  mca::core::Status discover(Topology& output) noexcept override {
    if (state_ != EthercatMasterState::Offline)
      return {mca::core::StatusCode::RuntimeError};
    Topology discovered;
    discovered.slave_count = 1U;
    discovered.sequence = 1U;
    discovered.slaves[0].ring_position = 0U;
    discovered.slaves[0].vendor_id = 0x00000002U;
    discovered.slaves[0].product_code = 0x1234U;
    discovered.slaves[0].al_state = EthercatAlState::PreOperational;
    output = discovered;
    state_ = EthercatMasterState::Discovered;
    return {};
  }

  mca::core::Status configure(const Layout& layout) noexcept override {
    if (state_ != EthercatMasterState::Discovered ||
        !layout.valid(1U))
      return {mca::core::StatusCode::InvalidArgument};
    layout_ = layout;
    state_ = EthercatMasterState::Configured;
    return {};
  }

  mca::core::Status activate() noexcept override {
    if (state_ != EthercatMasterState::Configured)
      return {mca::core::StatusCode::RuntimeError};
    state_ = EthercatMasterState::Active;
    return {};
  }

  mca::core::Status deactivate() noexcept override {
    if (state_ != EthercatMasterState::Active)
      return {mca::core::StatusCode::RuntimeError};
    state_ = EthercatMasterState::Configured;
    return {};
  }

  mca::core::Status exchange(std::uint16_t domain_id, const ProcessImage& output,
                             ProcessImage& input,
                             EthercatDomainCycleStatus& cycle_status) noexcept override {
    if (state_ != EthercatMasterState::Active || !output.valid() ||
        domain_id != layout_.domains[0].id)
      return {mca::core::StatusCode::RuntimeError};
    ProcessImage next_input;
    next_input.size = 1U;
    next_input.bytes[0] = output.size > 0U ? output.bytes[0] : 0U;
    EthercatDomainCycleStatus next_status;
    next_status.domain_id = domain_id;
    next_status.expected_working_counter = 3U;
    next_status.observed_working_counter = 3U;
    next_status.working_counter_state = EthercatWorkingCounterState::Complete;
    next_status.cycle_sequence = ++cycle_;
    next_status.timestamp_ns = 1000U + cycle_;
    next_status.deadline_timestamp_ns = 2000U + cycle_;
    next_status.completion_timestamp_ns = 1999U + cycle_;
    next_status.missed_cycles_since_previous_exchange = 0U;
    next_status.timing_state = EthercatCycleTimingState::OnTime;
    input = next_input;
    cycle_status = next_status;
    return {};
  }

  mca::core::Status clock_status(EthercatClockStatus& output) const noexcept override {
    if (state_ != EthercatMasterState::Active)
      return {mca::core::StatusCode::RuntimeError};
    output.state = EthercatDcState::Locked;
    output.cycle_time_ns = 1000000U;
    output.phase_error_ns = 120;
    return {};
  }

  EthercatMasterState state() const noexcept override { return state_; }

 private:
  EthercatMasterState state_{EthercatMasterState::Offline};
  Layout layout_{};
  std::uint64_t cycle_{0U};
};

class FakeEthercatMailbox final : public IEthercatMailbox<8U> {
 public:
  mca::core::Status enqueue(const Request& request) noexcept override {
    if (!request.valid()) return {mca::core::StatusCode::InvalidArgument};
    if (occupied_) return {mca::core::StatusCode::RuntimeError};
    request_ = request;
    result_ = {};
    result_.request_id = request.request_id;
    result_.state = EthercatMailboxResultState::Pending;
    occupied_ = true;
    return {};
  }

  mca::core::Status poll(std::uint64_t request_id,
                         Result& output) noexcept override {
    if (!occupied_ || request_id != request_.request_id)
      return {mca::core::StatusCode::InvalidArgument};
    output = result_;
    return {};
  }

  void complete(std::uint8_t value) noexcept {
    result_.state = EthercatMailboxResultState::Completed;
    result_.payload[0] = value;
    result_.payload_size = 1U;
  }

 private:
  Request request_{};
  Result result_{};
  bool occupied_{false};
};

EthercatProcessDataLayout<2U, 8U> make_valid_layout() {
  EthercatProcessDataLayout<2U, 8U> layout;
  layout.domain_count = 1U;
  layout.domains[0].id = 7U;
  layout.domains[0].input_bit_size = 16U;
  layout.domains[0].output_bit_size = 16U;
  layout.domains[0].expected_working_counter = 3U;
  layout.domains[0].cycle_time_ns = 1000000U;
  layout.domains[0].distributed_clocks_enabled = true;
  layout.entry_count = 2U;
  layout.entries[0] = {0U, 7U, 0x6041U, 0U, 16U,
                       EthercatPdoDirection::Input, 0U};
  layout.entries[1] = {0U, 7U, 0x6040U, 0U, 16U,
                       EthercatPdoDirection::Output, 0U};
  return layout;
}

TEST(EthercatTopology, RejectsEmptyDuplicateAndOverCapacitySnapshots) {
  EthercatTopologySnapshot<2U> topology;
  EXPECT_FALSE(topology.valid());
  topology.slave_count = 2U;
  topology.sequence = 1U;
  topology.slaves[0].ring_position = 0U;
  topology.slaves[1].ring_position = 0U;
  EXPECT_FALSE(topology.valid());
  topology.slaves[1].ring_position = 1U;
  EXPECT_TRUE(topology.valid());
  topology.slave_count = 3U;
  EXPECT_FALSE(topology.valid());
}

TEST(EthercatTopologyRecovery, RequiresExactIdentityBeforeLayoutReuse) {
  EthercatTopologySnapshot<3U> configured;
  configured.slave_count = 2U;
  configured.sequence = 1U;
  configured.slaves[0] = {0U, 0x02U, 0x1001U, 3U, 11U,
                          EthercatAlState::Operational, 0U};
  configured.slaves[1] = {1U, 0x02U, 0x1002U, 4U, 12U,
                          EthercatAlState::Operational, 0U};
  ASSERT_TRUE(configured.valid());

  auto rediscovered = configured;
  rediscovered.sequence = 2U;
  rediscovered.slaves[0].al_state = EthercatAlState::PreOperational;
  EXPECT_TRUE(ethercat_topology_identity_matches(configured, rediscovered));

  rediscovered.slaves[1].serial_number++;
  EXPECT_FALSE(ethercat_topology_identity_matches(configured, rediscovered));
  rediscovered = configured;
  rediscovered.slave_count = 1U;
  EXPECT_FALSE(ethercat_topology_identity_matches(configured, rediscovered));
  rediscovered = configured;
  rediscovered.slaves[0].ring_position = 2U;
  EXPECT_FALSE(ethercat_topology_identity_matches(configured, rediscovered));
  rediscovered = configured;
  rediscovered.slaves[0].revision++;
  EXPECT_FALSE(ethercat_topology_identity_matches(configured, rediscovered));
}

TEST(EthercatSlaveRecovery, RequiresExplicitRecoveringBeforeReturningPresent) {
  EXPECT_TRUE(ethercat_slave_health_transition_allowed(
      EthercatSlaveHealthState::Present, EthercatSlaveHealthState::Lost));
  EXPECT_TRUE(ethercat_slave_health_transition_allowed(
      EthercatSlaveHealthState::Lost, EthercatSlaveHealthState::Recovering));
  EXPECT_TRUE(ethercat_slave_health_transition_allowed(
      EthercatSlaveHealthState::Recovering, EthercatSlaveHealthState::Present));
  EXPECT_TRUE(ethercat_slave_health_transition_allowed(
      EthercatSlaveHealthState::Fault, EthercatSlaveHealthState::Recovering));
  EXPECT_FALSE(ethercat_slave_health_transition_allowed(
      EthercatSlaveHealthState::Lost, EthercatSlaveHealthState::Present));
  EXPECT_FALSE(ethercat_slave_health_transition_allowed(
      EthercatSlaveHealthState::Present, EthercatSlaveHealthState::Recovering));
  EXPECT_FALSE(ethercat_slave_health_transition_allowed(
      EthercatSlaveHealthState::Fault, EthercatSlaveHealthState::Present));
  EXPECT_TRUE(ethercat_slave_health_transition_allowed(
      EthercatSlaveHealthState::Unknown, EthercatSlaveHealthState::Present));
}

TEST(EthercatSlaveRecovery, ValidatesBoundedUniqueHealthSnapshots) {
  EthercatSlaveHealthSnapshot<2U> health;
  EXPECT_FALSE(health.valid());
  health.slave_count = 2U;
  health.sequence = 1U;
  health.slaves[0] = {0U, EthercatSlaveHealthState::Present};
  health.slaves[1] = {1U, EthercatSlaveHealthState::Lost};
  EXPECT_TRUE(health.valid());
  health.slaves[1].ring_position = 0U;
  EXPECT_FALSE(health.valid());
  health.slaves[1] = {1U, EthercatSlaveHealthState::Unknown};
  EXPECT_FALSE(health.valid());
  health.slave_count = 3U;
  EXPECT_FALSE(health.valid());
}

TEST(EthercatPdoLayout, AcceptsBitPreciseMappingsAndRejectsInvalidRanges) {
  auto layout = make_valid_layout();
  EXPECT_TRUE(layout.valid(1U));
  layout.entries[1].direction = EthercatPdoDirection::Input;
  layout.entries[1].bit_offset = 1U;
  layout.entries[1].bit_length = 8U;
  EXPECT_FALSE(layout.valid(1U));
  layout = make_valid_layout();
  layout.entries[0].bit_offset = 1U;
  layout.entries[0].bit_length = 16U;
  EXPECT_FALSE(layout.valid(1U));
  layout = make_valid_layout();
  layout.entries[0].slave_position = 1U;
  EXPECT_FALSE(layout.valid(1U));
  layout = make_valid_layout();
  layout.entries[0].domain_id = 9U;
  EXPECT_FALSE(layout.valid(1U));
  layout = make_valid_layout();
  layout.domains[0].distributed_clocks_enabled = true;
  layout.domains[0].cycle_time_ns = 0U;
  EXPECT_FALSE(layout.valid(1U));
}

TEST(EthercatPdoLayout, RejectsPartialOverlapAndAcceptsAdjacentBitRanges) {
  auto layout = make_valid_layout();
  layout.domains[0].input_bit_size = 24U;
  layout.entries[0].bit_length = 11U;
  layout.entries[1].direction = EthercatPdoDirection::Input;
  layout.entries[1].bit_offset = 10U;
  layout.entries[1].bit_length = 13U;
  EXPECT_FALSE(layout.valid(1U));

  // Half-open ranges [0, 11) and [11, 24) are adjacent, not overlapping.
  layout.entries[1].bit_offset = 11U;
  EXPECT_TRUE(layout.valid(1U));

  // The last bit may end exactly at the declared domain boundary.
  layout.domains[0].input_bit_size = 23U;
  EXPECT_FALSE(layout.valid(1U));
}

TEST(EthercatPdoLayout, RejectsDuplicateDomainIdsAndOverlappingEntries) {
  auto layout = make_valid_layout();
  layout.domain_count = 2U;
  layout.domains[1] = layout.domains[0];
  EXPECT_FALSE(layout.valid(1U));
  layout = make_valid_layout();
  layout.entry_count = 2U;
  layout.entries[1] = layout.entries[0];
  EXPECT_FALSE(layout.valid(1U));
}

TEST(EthercatWorkingCounter, ClassifiesExpectedAndObservedValues) {
  EXPECT_EQ(classify_ethercat_working_counter(0U, 0U),
            EthercatWorkingCounterState::Unknown);
  EXPECT_EQ(classify_ethercat_working_counter(3U, 0U),
            EthercatWorkingCounterState::Zero);
  EXPECT_EQ(classify_ethercat_working_counter(3U, 2U),
            EthercatWorkingCounterState::Incomplete);
  EXPECT_EQ(classify_ethercat_working_counter(3U, 3U),
            EthercatWorkingCounterState::Complete);
  EXPECT_EQ(classify_ethercat_working_counter(3U, 4U),
            EthercatWorkingCounterState::Unexpected);
}

TEST(EthercatCycleTiming, ClassifiesDeadlineBoundaryAndMissedCycles) {
  EXPECT_EQ(classify_ethercat_cycle_timing(100U, 0U, 0U),
            EthercatCycleTimingState::Unknown);
  EXPECT_EQ(classify_ethercat_cycle_timing(100U, 100U, 0U),
            EthercatCycleTimingState::OnTime);
  EXPECT_EQ(classify_ethercat_cycle_timing(100U, 99U, 0U),
            EthercatCycleTimingState::OnTime);
  EXPECT_EQ(classify_ethercat_cycle_timing(100U, 101U, 0U),
            EthercatCycleTimingState::Late);
  EXPECT_EQ(classify_ethercat_cycle_timing(100U, 90U, 1U),
            EthercatCycleTimingState::Missed);
}

TEST(EthercatCycleTiming, StatusValidatesTimingClassification) {
  EthercatDomainCycleStatus status;
  status.domain_id = 7U;
  status.expected_working_counter = 3U;
  status.observed_working_counter = 3U;
  status.working_counter_state = EthercatWorkingCounterState::Complete;
  status.cycle_sequence = 1U;
  status.timestamp_ns = 1000U;
  status.deadline_timestamp_ns = 2000U;
  status.completion_timestamp_ns = 1999U;
  status.timing_state = EthercatCycleTimingState::OnTime;
  EXPECT_TRUE(status.valid());

  status.timing_state = EthercatCycleTimingState::Late;
  EXPECT_FALSE(status.valid());
  status.completion_timestamp_ns = 2001U;
  EXPECT_TRUE(status.valid());

  status.missed_cycles_since_previous_exchange = 1U;
  status.timing_state = EthercatCycleTimingState::Missed;
  EXPECT_TRUE(status.valid());
}

TEST(EthercatAlContract, AcceptsLegalProgressionFallbackAndErrorRecovery) {
  EXPECT_TRUE(ethercat_al_transition_allowed(EthercatAlState::Init,
                                              EthercatAlState::Init));
  EXPECT_TRUE(ethercat_al_transition_allowed(EthercatAlState::Init,
                                              EthercatAlState::PreOperational));
  EXPECT_TRUE(ethercat_al_transition_allowed(EthercatAlState::Init,
                                              EthercatAlState::Bootstrap));
  EXPECT_TRUE(ethercat_al_transition_allowed(
      EthercatAlState::PreOperational, EthercatAlState::SafeOperational));
  EXPECT_TRUE(ethercat_al_transition_allowed(
      EthercatAlState::SafeOperational, EthercatAlState::Operational));
  EXPECT_TRUE(ethercat_al_transition_allowed(
      EthercatAlState::Operational, EthercatAlState::SafeOperational));
  EXPECT_TRUE(ethercat_al_transition_allowed(EthercatAlState::SafeOperational,
                                              EthercatAlState::PreOperational));
  EXPECT_TRUE(ethercat_al_transition_allowed(EthercatAlState::PreOperational,
                                              EthercatAlState::Init));
  EXPECT_TRUE(ethercat_al_transition_allowed(EthercatAlState::Operational,
                                              EthercatAlState::Init));
  EXPECT_TRUE(ethercat_al_transition_allowed(EthercatAlState::Bootstrap,
                                              EthercatAlState::Init));
  EXPECT_TRUE(ethercat_al_transition_allowed(EthercatAlState::Error,
                                              EthercatAlState::Init));
  EXPECT_TRUE(ethercat_al_transition_allowed(EthercatAlState::Operational,
                                              EthercatAlState::Error));
}

TEST(EthercatAlContract, RejectsIllegalTransitionsAndUnknownRecoveryTargets) {
  EXPECT_FALSE(ethercat_al_transition_allowed(EthercatAlState::Unknown,
                                               EthercatAlState::Init));
  EXPECT_FALSE(ethercat_al_transition_allowed(EthercatAlState::Init,
                                               EthercatAlState::SafeOperational));
  EXPECT_FALSE(ethercat_al_transition_allowed(EthercatAlState::Init,
                                               EthercatAlState::Operational));
  EXPECT_FALSE(ethercat_al_transition_allowed(EthercatAlState::PreOperational,
                                               EthercatAlState::Operational));
  EXPECT_FALSE(ethercat_al_transition_allowed(EthercatAlState::Operational,
                                               EthercatAlState::Bootstrap));
  EXPECT_FALSE(ethercat_al_transition_allowed(EthercatAlState::Bootstrap,
                                               EthercatAlState::PreOperational));
  EXPECT_FALSE(ethercat_al_transition_allowed(EthercatAlState::Error,
                                               EthercatAlState::PreOperational));
  EXPECT_FALSE(ethercat_al_transition_allowed(EthercatAlState::Error,
                                               EthercatAlState::Operational));
}

TEST(EthercatDcContract, AcceptsHealthProgressionAndRecovery) {
  EXPECT_TRUE(ethercat_dc_transition_allowed(EthercatDcState::Unsupported,
                                              EthercatDcState::Unsupported));
  EXPECT_TRUE(ethercat_dc_transition_allowed(EthercatDcState::NotConfigured,
                                              EthercatDcState::Synchronizing));
  EXPECT_TRUE(ethercat_dc_transition_allowed(EthercatDcState::Synchronizing,
                                              EthercatDcState::Locked));
  EXPECT_TRUE(ethercat_dc_transition_allowed(EthercatDcState::Locked,
                                              EthercatDcState::Degraded));
  EXPECT_TRUE(ethercat_dc_transition_allowed(EthercatDcState::Degraded,
                                              EthercatDcState::Locked));
  EXPECT_TRUE(ethercat_dc_transition_allowed(EthercatDcState::Locked,
                                              EthercatDcState::Lost));
  EXPECT_TRUE(ethercat_dc_transition_allowed(EthercatDcState::Lost,
                                              EthercatDcState::Synchronizing));
  EXPECT_TRUE(ethercat_dc_transition_allowed(EthercatDcState::Synchronizing,
                                              EthercatDcState::NotConfigured));
}

TEST(EthercatDcContract, RejectsImpossibleHealthTransitions) {
  EXPECT_FALSE(ethercat_dc_transition_allowed(EthercatDcState::Unsupported,
                                               EthercatDcState::NotConfigured));
  EXPECT_FALSE(ethercat_dc_transition_allowed(EthercatDcState::NotConfigured,
                                               EthercatDcState::Locked));
  EXPECT_FALSE(ethercat_dc_transition_allowed(EthercatDcState::NotConfigured,
                                               EthercatDcState::Lost));
  EXPECT_FALSE(ethercat_dc_transition_allowed(EthercatDcState::Lost,
                                               EthercatDcState::Locked));
  EXPECT_FALSE(ethercat_dc_transition_allowed(EthercatDcState::Lost,
                                               EthercatDcState::Degraded));
}

TEST(EthercatMailboxContract, ValidatesBoundedReadWriteRequestsAndResults) {
  EthercatSdoRequest<8U> request;
  EXPECT_FALSE(request.valid());
  request.request_id = 1U;
  request.slave_position = 0U;
  request.index = 0x6041U;
  request.operation = EthercatSdoOperation::Read;
  request.max_response_size = 2U;
  request.deadline_timestamp_ns = 1000U;
  EXPECT_TRUE(request.valid());
  request.payload_size = 1U;
  EXPECT_FALSE(request.valid());
  request.payload_size = 0U;
  request.operation = EthercatSdoOperation::Write;
  request.payload_size = 2U;
  request.max_response_size = 0U;
  EXPECT_TRUE(request.valid());
  request.payload_size = 9U;
  EXPECT_FALSE(request.valid());

  EthercatSdoResult<8U> result;
  EXPECT_FALSE(result.valid());
  result.request_id = 1U;
  result.state = EthercatMailboxResultState::Pending;
  EXPECT_TRUE(result.valid());
  result.state = EthercatMailboxResultState::Completed;
  result.payload_size = 2U;
  EXPECT_TRUE(result.valid());
  result.abort_code = 0x06020000U;
  EXPECT_FALSE(result.valid());
  result.state = EthercatMailboxResultState::Aborted;
  result.payload_size = 0U;
  EXPECT_TRUE(result.valid());
}

TEST(EthercatMailboxContract, FakeMailboxIsBoundedAndPolledWithoutBlocking) {
  FakeEthercatMailbox mailbox;
  EthercatSdoRequest<8U> request;
  request.request_id = 9U;
  request.slave_position = 0U;
  request.index = 0x6041U;
  request.operation = EthercatSdoOperation::Read;
  request.max_response_size = 2U;
  request.deadline_timestamp_ns = 5000U;
  ASSERT_TRUE(mailbox.enqueue(request).ok());

  EthercatSdoResult<8U> result;
  ASSERT_TRUE(mailbox.poll(9U, result).ok());
  EXPECT_EQ(result.state, EthercatMailboxResultState::Pending);
  EXPECT_EQ(mailbox.enqueue(request).code, mca::core::StatusCode::RuntimeError);
  EXPECT_EQ(mailbox.poll(10U, result).code,
            mca::core::StatusCode::InvalidArgument);

  mailbox.complete(0x34U);
  ASSERT_TRUE(mailbox.poll(9U, result).ok());
  EXPECT_TRUE(result.valid());
  EXPECT_EQ(result.state, EthercatMailboxResultState::Completed);
  EXPECT_EQ(result.payload[0], 0x34U);
}

TEST(EthercatMasterContract, FakeMasterExercisesLifecycleAndCyclicExchange) {
  FakeEthercatMaster master;
  EXPECT_EQ(master.state(), EthercatMasterState::Offline);
  EXPECT_EQ(master.activate().code, mca::core::StatusCode::RuntimeError);

  TestMaster::Topology topology;
  ASSERT_TRUE(master.discover(topology).ok());
  ASSERT_TRUE(topology.valid());
  EXPECT_EQ(master.state(), EthercatMasterState::Discovered);

  auto layout = make_valid_layout();
  ASSERT_TRUE(master.configure(layout).ok());
  ASSERT_TRUE(master.activate().ok());
  EXPECT_EQ(master.state(), EthercatMasterState::Active);

  TestMaster::ProcessImage output;
  output.size = 1U;
  output.bytes[0] = 0x5AU;
  TestMaster::ProcessImage input;
  EthercatDomainCycleStatus cycle;
  ASSERT_TRUE(master.exchange(7U, output, input, cycle).ok());
  EXPECT_EQ(input.bytes[0], 0x5AU);
  EXPECT_TRUE(cycle.valid());
  EXPECT_EQ(cycle.working_counter_state,
            EthercatWorkingCounterState::Complete);
  EXPECT_EQ(cycle.timing_state, EthercatCycleTimingState::OnTime);

  EthercatClockStatus clock;
  ASSERT_TRUE(master.clock_status(clock).ok());
  EXPECT_EQ(clock.state, EthercatDcState::Locked);
  EXPECT_EQ(clock.cycle_time_ns, 1000000U);

  ASSERT_TRUE(master.deactivate().ok());
  EXPECT_EQ(master.state(), EthercatMasterState::Configured);
}

TEST(EthercatMasterContract, FailedOperationsPreserveCallerOutputs) {
  FakeEthercatMaster master;
  TestMaster::Topology topology;
  ASSERT_TRUE(master.discover(topology).ok());
  ASSERT_TRUE(master.configure(make_valid_layout()).ok());
  ASSERT_TRUE(master.activate().ok());

  TestMaster::ProcessImage input;
  input.size = 1U;
  input.bytes[0] = 0xA5U;
  const auto input_before = input;

  EthercatDomainCycleStatus cycle;
  cycle.domain_id = 99U;
  cycle.expected_working_counter = 11U;
  cycle.observed_working_counter = 7U;
  cycle.working_counter_state = EthercatWorkingCounterState::Incomplete;
  cycle.cycle_sequence = 42U;
  cycle.timestamp_ns = 123456U;
  cycle.deadline_timestamp_ns = 234567U;
  cycle.completion_timestamp_ns = 234568U;
  cycle.missed_cycles_since_previous_exchange = 2U;
  cycle.timing_state = EthercatCycleTimingState::Missed;
  const auto cycle_before = cycle;

  TestMaster::ProcessImage output;
  output.size = 1U;
  output.bytes[0] = 0x11U;

  EXPECT_EQ(master.exchange(99U, output, input, cycle).code,
            mca::core::StatusCode::RuntimeError);
  EXPECT_EQ(input.size, input_before.size);
  EXPECT_EQ(input.bytes, input_before.bytes);
  EXPECT_EQ(cycle.domain_id, cycle_before.domain_id);
  EXPECT_EQ(cycle.expected_working_counter,
            cycle_before.expected_working_counter);
  EXPECT_EQ(cycle.observed_working_counter,
            cycle_before.observed_working_counter);
  EXPECT_EQ(cycle.working_counter_state, cycle_before.working_counter_state);
  EXPECT_EQ(cycle.cycle_sequence, cycle_before.cycle_sequence);
  EXPECT_EQ(cycle.timestamp_ns, cycle_before.timestamp_ns);
  EXPECT_EQ(cycle.deadline_timestamp_ns, cycle_before.deadline_timestamp_ns);
  EXPECT_EQ(cycle.completion_timestamp_ns, cycle_before.completion_timestamp_ns);
  EXPECT_EQ(cycle.missed_cycles_since_previous_exchange,
            cycle_before.missed_cycles_since_previous_exchange);
  EXPECT_EQ(cycle.timing_state, cycle_before.timing_state);

  EXPECT_EQ(master.deactivate().ok(), true);
  EXPECT_EQ(master.deactivate().code, mca::core::StatusCode::RuntimeError);
}

TEST(EthercatSdoProfile, EnforcesFixedCapacityAndIndependentConcurrencySlots) {
  EthercatSdoProfile<3U, 8U> profile;
  EXPECT_FALSE(profile.valid());

  profile.slot_count = 2U;
  profile.slots[0] = {2U, 0x6041U, 0U, 4U, 250U};
  profile.slots[1] = {2U, 0x6041U, 0U, 4U, 250U};
  ASSERT_TRUE(profile.valid());
  EXPECT_EQ(profile.concurrency(2U, 0x6041U, 0U), 2U);
  EXPECT_EQ(profile.concurrency(2U, 0x6064U, 0U), 0U);

  EthercatSdoRequest<8U> read;
  read.request_id = 1U;
  read.slave_position = 2U;
  read.index = 0x6041U;
  read.operation = EthercatSdoOperation::Read;
  read.max_response_size = 4U;
  read.deadline_timestamp_ns = 1000U;
  EXPECT_TRUE(profile.supports(read));
  read.max_response_size = 5U;
  EXPECT_FALSE(profile.supports(read));

  EthercatSdoRequest<8U> write;
  write.request_id = 2U;
  write.slave_position = 2U;
  write.index = 0x6041U;
  write.operation = EthercatSdoOperation::Write;
  write.payload_size = 4U;
  write.deadline_timestamp_ns = 1000U;
  EXPECT_TRUE(profile.supports(write));
  write.payload_size = 3U;
  EXPECT_FALSE(profile.supports(write));
  write.payload_size = 4U;
  write.index = 0x6064U;
  EXPECT_FALSE(profile.supports(write));
}

TEST(EthercatSdoProfile, RejectsInconsistentDuplicateSlotDefinitions) {
  EthercatSdoProfile<2U, 8U> profile;
  profile.slot_count = 2U;
  profile.slots[0] = {0U, 0x6040U, 0U, 2U, 100U};
  profile.slots[1] = {0U, 0x6040U, 0U, 4U, 100U};
  EXPECT_FALSE(profile.valid());

  profile.slots[1].transfer_capacity = 2U;
  profile.slots[1].timeout_ms = 0U;
  EXPECT_FALSE(profile.valid());
}

}  // namespace
