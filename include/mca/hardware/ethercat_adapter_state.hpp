#pragma once

#include <cstddef>
#include <cstdint>

#include "mca/core/types.hpp"
#include "mca/hardware/ethercat_master.hpp"

namespace mca::hardware {

template <std::size_t MaxSlaves, std::size_t MaxDomains,
          std::size_t MaxEntries>
class EthercatAdapterState final {
 public:
  using Topology = EthercatTopologySnapshot<MaxSlaves>;
  using Layout = EthercatProcessDataLayout<MaxDomains, MaxEntries>;

  [[nodiscard]] EthercatMasterState state() const noexcept { return state_; }
  [[nodiscard]] const Topology& topology() const noexcept { return topology_; }
  [[nodiscard]] const Layout& layout() const noexcept { return layout_; }

  mca::core::Status on_discovered(const Topology& topology) noexcept {
    if (state_ != EthercatMasterState::Offline || !topology.valid())
      return {mca::core::StatusCode::InvalidArgument};
    topology_ = topology;
    state_ = EthercatMasterState::Discovered;
    return {};
  }

  mca::core::Status on_configured(const Layout& layout) noexcept {
    if (state_ != EthercatMasterState::Discovered ||
        !layout.valid(topology_.slave_count))
      return {mca::core::StatusCode::InvalidArgument};
    layout_ = layout;
    state_ = EthercatMasterState::Configured;
    return {};
  }

  mca::core::Status on_activated() noexcept {
    if (state_ != EthercatMasterState::Configured)
      return {mca::core::StatusCode::RuntimeError};
    state_ = EthercatMasterState::Active;
    return {};
  }

  mca::core::Status on_deactivated() noexcept {
    if (state_ != EthercatMasterState::Active &&
        state_ != EthercatMasterState::Recovering)
      return {mca::core::StatusCode::RuntimeError};
    state_ = EthercatMasterState::Configured;
    return {};
  }

  mca::core::Status on_shutdown() noexcept {
    if (state_ == EthercatMasterState::Active ||
        state_ == EthercatMasterState::Recovering)
      return {mca::core::StatusCode::RuntimeError};
    state_ = EthercatMasterState::Offline;
    topology_ = Topology{};
    layout_ = Layout{};
    return {};
  }

  mca::core::Status begin_recovery() noexcept {
    if (state_ != EthercatMasterState::Active)
      return {mca::core::StatusCode::RuntimeError};
    state_ = EthercatMasterState::Recovering;
    return {};
  }

  mca::core::Status accept_recovered_topology(
      const Topology& rediscovered) noexcept {
    if (state_ != EthercatMasterState::Recovering || !rediscovered.valid())
      return {mca::core::StatusCode::InvalidArgument};
    if (!ethercat_topology_identity_matches(topology_, rediscovered)) {
      state_ = EthercatMasterState::Fault;
      return {mca::core::StatusCode::ConstraintViolation};
    }
    topology_ = rediscovered;
    state_ = EthercatMasterState::Configured;
    return {};
  }

  mca::core::Status fail() noexcept {
    if (state_ == EthercatMasterState::Offline)
      return {mca::core::StatusCode::RuntimeError};
    state_ = EthercatMasterState::Fault;
    return {};
  }

  mca::core::Status clear_fault() noexcept {
    if (state_ != EthercatMasterState::Fault)
      return {mca::core::StatusCode::RuntimeError};
    state_ = EthercatMasterState::Recovering;
    return {};
  }

 private:
  EthercatMasterState state_{EthercatMasterState::Offline};
  Topology topology_{};
  Layout layout_{};
};

}  // namespace mca::hardware
