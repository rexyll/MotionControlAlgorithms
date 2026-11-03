#include <cassert>

#include "mca/hardware/ethercat_adapter_state.hpp"

using Adapter = mca::hardware::EthercatAdapterState<2U, 1U, 2U>;

int main() {
  Adapter adapter;
  Adapter::Topology topology;
  topology.slave_count = 1U;
  topology.sequence = 1U;
  topology.slaves[0].ring_position = 0U;
  topology.slaves[0].vendor_id = 2U;
  topology.slaves[0].product_code = 0x1234U;
  topology.slaves[0].revision = 1U;
  topology.slaves[0].serial_number = 7U;
  topology.slaves[0].al_state = mca::hardware::EthercatAlState::PreOperational;

  Adapter::Layout layout;
  layout.domain_count = 1U;
  layout.domains[0].id = 1U;
  layout.domains[0].input_bit_size = 16U;
  layout.domains[0].output_bit_size = 16U;
  layout.domains[0].expected_working_counter = 3U;
  layout.entry_count = 2U;
  layout.entries[0] = {0U, 1U, 0x6041U, 0U, 16U,
                       mca::hardware::EthercatPdoDirection::Input, 0U};
  layout.entries[1] = {0U, 1U, 0x6040U, 0U, 16U,
                       mca::hardware::EthercatPdoDirection::Output, 0U};

  assert(adapter.on_discovered(topology).ok());
  assert(adapter.on_configured(layout).ok());
  assert(adapter.on_activated().ok());
  assert(adapter.begin_recovery().ok());

  auto changed = topology;
  changed.sequence = 2U;
  changed.slaves[0].serial_number = 8U;
  assert(adapter.accept_recovered_topology(changed).code ==
         mca::core::StatusCode::ConstraintViolation);
  assert(adapter.state() == mca::hardware::EthercatMasterState::Fault);

  assert(adapter.clear_fault().ok());
  assert(adapter.state() == mca::hardware::EthercatMasterState::Recovering);
  return 0;
}
