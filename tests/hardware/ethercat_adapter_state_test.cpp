#include <cstdint>

#include <gtest/gtest.h>

#include "mca/hardware/ethercat_adapter_state.hpp"

namespace {

using namespace mca::hardware;
using Adapter = EthercatAdapterState<4U, 2U, 8U>;

Adapter::Topology make_topology() {
  Adapter::Topology topology;
  topology.slave_count = 1U;
  topology.sequence = 1U;
  topology.slaves[0].ring_position = 0U;
  topology.slaves[0].vendor_id = 0x00000002U;
  topology.slaves[0].product_code = 0x1234U;
  topology.slaves[0].revision = 3U;
  topology.slaves[0].serial_number = 7U;
  topology.slaves[0].al_state = EthercatAlState::PreOperational;
  return topology;
}

Adapter::Layout make_layout() {
  Adapter::Layout layout;
  layout.domain_count = 1U;
  layout.domains[0].id = 1U;
  layout.domains[0].input_bit_size = 16U;
  layout.domains[0].output_bit_size = 16U;
  layout.domains[0].expected_working_counter = 3U;
  layout.domains[0].cycle_time_ns = 1000000U;
  layout.entry_count = 2U;
  layout.entries[0] = {0U, 1U, 0x6041U, 0U, 16U,
                       EthercatPdoDirection::Input, 0U};
  layout.entries[1] = {0U, 1U, 0x6040U, 0U, 16U,
                       EthercatPdoDirection::Output, 0U};
  return layout;
}

TEST(EthercatAdapterState, EnforcesLifecycleOrdering) {
  Adapter adapter;
  const auto topology = make_topology();
  const auto layout = make_layout();

  EXPECT_EQ(adapter.state(), EthercatMasterState::Offline);
  EXPECT_EQ(adapter.on_activated().code, mca::core::StatusCode::RuntimeError);
  EXPECT_TRUE(adapter.on_discovered(topology).ok());
  EXPECT_EQ(adapter.state(), EthercatMasterState::Discovered);
  EXPECT_EQ(adapter.on_activated().code, mca::core::StatusCode::RuntimeError);
  EXPECT_TRUE(adapter.on_configured(layout).ok());
  EXPECT_EQ(adapter.state(), EthercatMasterState::Configured);
  EXPECT_TRUE(adapter.on_activated().ok());
  EXPECT_EQ(adapter.state(), EthercatMasterState::Active);
  EXPECT_TRUE(adapter.on_deactivated().ok());
  EXPECT_EQ(adapter.state(), EthercatMasterState::Configured);
}

TEST(EthercatAdapterState, RejectsInvalidDiscoveryAndConfiguration) {
  Adapter adapter;
  Adapter::Topology invalid_topology;
  EXPECT_EQ(adapter.on_discovered(invalid_topology).code,
            mca::core::StatusCode::InvalidArgument);

  auto topology = make_topology();
  ASSERT_TRUE(adapter.on_discovered(topology).ok());
  auto invalid_layout = make_layout();
  invalid_layout.domains[0].id = 0U;
  EXPECT_EQ(adapter.on_configured(invalid_layout).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_EQ(adapter.state(), EthercatMasterState::Discovered);
}

TEST(EthercatAdapterState, IdentityMismatchDuringRecoveryCausesFault) {
  Adapter adapter;
  ASSERT_TRUE(adapter.on_discovered(make_topology()).ok());
  ASSERT_TRUE(adapter.on_configured(make_layout()).ok());
  ASSERT_TRUE(adapter.on_activated().ok());
  ASSERT_TRUE(adapter.begin_recovery().ok());

  auto changed = make_topology();
  changed.sequence = 2U;
  changed.slaves[0].serial_number++;
  EXPECT_EQ(adapter.accept_recovered_topology(changed).code,
            mca::core::StatusCode::ConstraintViolation);
  EXPECT_EQ(adapter.state(), EthercatMasterState::Fault);
}

TEST(EthercatAdapterState, MatchingRecoveryInvalidatesOldConfigurationState) {
  Adapter adapter;
  auto topology = make_topology();
  ASSERT_TRUE(adapter.on_discovered(topology).ok());
  ASSERT_TRUE(adapter.on_configured(make_layout()).ok());
  ASSERT_TRUE(adapter.on_activated().ok());
  ASSERT_TRUE(adapter.begin_recovery().ok());

  topology.sequence = 2U;
  topology.slaves[0].al_state = EthercatAlState::SafeOperational;
  ASSERT_TRUE(adapter.accept_recovered_topology(topology).ok());
  EXPECT_EQ(adapter.state(), EthercatMasterState::Configured);
  EXPECT_EQ(adapter.topology().sequence, 2U);

  // Recovery returns to Configured, never directly to Active. A fresh backend
  // activation is required after identity/PDO validation has completed.
  ASSERT_TRUE(adapter.on_activated().ok());
  EXPECT_EQ(adapter.state(), EthercatMasterState::Active);
}

TEST(EthercatAdapterState, FaultCanOnlyReturnThroughRecovery) {
  Adapter adapter;
  ASSERT_TRUE(adapter.on_discovered(make_topology()).ok());
  ASSERT_TRUE(adapter.on_configured(make_layout()).ok());
  ASSERT_TRUE(adapter.on_activated().ok());
  ASSERT_TRUE(adapter.fail().ok());
  EXPECT_EQ(adapter.state(), EthercatMasterState::Fault);
  EXPECT_EQ(adapter.on_activated().code, mca::core::StatusCode::RuntimeError);
  ASSERT_TRUE(adapter.clear_fault().ok());
  EXPECT_EQ(adapter.state(), EthercatMasterState::Recovering);
  EXPECT_EQ(adapter.on_deactivated().code, mca::core::StatusCode::RuntimeError);
}

}  // namespace
