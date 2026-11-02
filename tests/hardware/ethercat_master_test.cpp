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
    discovered.slaves[0].station_position = 0U;
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
  topology.slaves[0].station_position = 0U;
  topology.slaves[1].station_position = 0U;
  EXPECT_FALSE(topology.valid());
  topology.slaves[1].station_position = 1U;
  EXPECT_TRUE(topology.valid());
  topology.slave_count = 3U;
  EXPECT_FALSE(topology.valid());
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

TEST(EthercatMasterContract, FakeMasterExercisesLifecycleAndCyclicExchange) {
  FakeEthercatMaster master;
  EXPECT_EQ(master.state(), EthercatMasterState::Offline);
  EXPECT_EQ(master.activate().code, mca::mca::core::StatusCode::RuntimeError);

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

  EthercatClockStatus clock;
  ASSERT_TRUE(master.clock_status(clock).ok());
  EXPECT_EQ(clock.state, EthercatDcState::Locked);
  EXPECT_EQ(clock.cycle_time_ns, 1000000U);

  ASSERT_TRUE(master.deactivate().ok());
  EXPECT_EQ(master.state(), EthercatMasterState::Configured);
}

}  // namespace
