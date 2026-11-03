#include "mca/hardware/ethercat_igh_adapter.hpp"

#if defined(MCA_ENABLE_IGH)
#include <algorithm>

namespace mca::hardware {

template <std::size_t S, std::size_t D, std::size_t E>
EthercatAlState EthercatIghAdapter<S, D, E>::map_al_state(std::uint8_t state) noexcept {
  switch (state & 0x0FU) {
    case EC_AL_STATE_INIT: return EthercatAlState::Init;
    case EC_AL_STATE_PREOP: return EthercatAlState::PreOperational;
    case 0x03U: return EthercatAlState::Bootstrap;
    case EC_AL_STATE_SAFEOP: return EthercatAlState::SafeOperational;
    case EC_AL_STATE_OP: return EthercatAlState::Operational;
    default: return EthercatAlState::Unknown;
  }
}

template <std::size_t S, std::size_t D, std::size_t E>
mca::core::Status EthercatIghAdapter<S, D, E>::request_master() noexcept {
  if (master_ != nullptr || state_.state() != EthercatMasterState::Offline)
    return {mca::core::StatusCode::RuntimeError};
  master_ = ecrt_request_master(config_.master_index);
  return master_ != nullptr ? mca::core::Status{} : mca::core::Status{mca::core::StatusCode::RuntimeError};
}

template <std::size_t S, std::size_t D, std::size_t E>
mca::core::Status EthercatIghAdapter<S, D, E>::discover() noexcept {
  if (master_ == nullptr || state_.state() != EthercatMasterState::Offline)
    return {mca::core::StatusCode::RuntimeError};
  ec_master_info_t info{};
  if (ecrt_master(master_, &info) != 0 || info.slave_count == 0U || info.slave_count > S)
    return {mca::core::StatusCode::RuntimeError};
  Topology t{};
  t.slave_count = info.slave_count;
  t.sequence = 1U;
  for (std::size_t i = 0; i < t.slave_count; ++i) {
    ec_slave_info_t src{};
    if (ecrt_master_get_slave(master_, static_cast<std::uint16_t>(i), &src) != 0)
      return {mca::core::StatusCode::RuntimeError};
    auto& dst = t.slaves[i];
    dst.station_position = src.position;
    dst.vendor_id = src.vendor_id;
    dst.product_code = src.product_code;
    dst.revision = src.revision_number;
    dst.serial_number = src.serial_number;
    dst.al_state = map_al_state(src.al_state);
  }
  return state_.on_discovered(t);
}

template <std::size_t S, std::size_t D, std::size_t E>
mca::core::Status EthercatIghAdapter<S, D, E>::configure(
    const std::array<PdoBinding, E>& bindings, std::size_t count) noexcept {
  if (master_ == nullptr || state_.state() != EthercatMasterState::Discovered || count == 0U || count > E)
    return {mca::core::StatusCode::InvalidArgument};
  std::array<std::uint16_t, D> ids{};
  domain_count_ = 0U;
  binding_count_ = count;
  for (std::size_t i = 0; i < count; ++i) {
    const auto& b = bindings[i];
    if (b.station_position >= state_.topology().slave_count || b.index == 0U || b.bit_length == 0U || b.bit_length > 64U)
      return {mca::core::StatusCode::InvalidArgument};
    std::size_t di = domain_count_;
    for (std::size_t d = 0; d < domain_count_; ++d) if (ids[d] == b.domain_id) { di = d; break; }
    if (di == domain_count_) {
      if (domain_count_ >= D) return {mca::core::StatusCode::ConstraintViolation};
      ids[domain_count_] = b.domain_id;
      domains_[domain_count_] = ecrt_master_create_domain(master_);
      if (domains_[domain_count_] == nullptr) return {mca::core::StatusCode::RuntimeError};
      ++domain_count_;
    }
    const auto& slave = state_.topology().slaves[b.station_position];
    if (slave.station_position != b.station_position || slave.vendor_id != b.vendor_id || slave.product_code != b.product_code)
      return {mca::core::StatusCode::ConstraintViolation};
    if (slave_configs_[b.station_position] == nullptr)
      slave_configs_[b.station_position] = ecrt_master_slave_config(master_, 0U, b.station_position, b.vendor_id, b.product_code);
    if (slave_configs_[b.station_position] == nullptr) return {mca::core::StatusCode::RuntimeError};
    unsigned int bit = 0U;
    const int off = ecrt_slave_config_reg_pdo_entry(slave_configs_[b.station_position], b.index, b.subindex, domains_[di], &bit);
    if (off < 0) return {mca::core::StatusCode::RuntimeError};
    offsets_[i] = static_cast<unsigned int>(off);
    bit_positions_[i] = bit;
  }
  Layout l{};
  l.domain_count = domain_count_;
  l.entry_count = count;
  for (std::size_t d = 0; d < domain_count_; ++d) { l.domains[d].id = ids[d]; l.domains[d].cycle_time_ns = config_.cycle_time_ns; }
  for (std::size_t i = 0; i < count; ++i) {
    const auto& b = bindings[i];
    auto& e = l.entries[i];
    e.slave_position = b.station_position; e.domain_id = b.domain_id; e.index = b.index; e.subindex = b.subindex;
    e.bit_length = b.bit_length; e.direction = b.direction;
    e.bit_offset = offsets_[i] * 8U + bit_positions_[i];
    for (std::size_t d = 0; d < domain_count_; ++d) if (l.domains[d].id == b.domain_id) {
      const std::uint32_t end = e.bit_offset + b.bit_length;
      if (b.direction == EthercatPdoDirection::Input) l.domains[d].input_bit_size = std::max(l.domains[d].input_bit_size, end);
      else l.domains[d].output_bit_size = std::max(l.domains[d].output_bit_size, end);
      break;
    }
  }
  return state_.on_configured(l);
}

template <std::size_t S, std::size_t D, std::size_t E>
mca::core::Status EthercatIghAdapter<S, D, E>::activate() noexcept {
  if (master_ == nullptr || state_.state() != EthercatMasterState::Configured)
    return {mca::core::StatusCode::RuntimeError};
  if (config_.send_interval_us != 0U && ecrt_master_set_send_interval(master_, config_.send_interval_us) != 0)
    return {mca::core::StatusCode::RuntimeError};
  if (ecrt_master_activate(master_) != 0) return {mca::core::StatusCode::RuntimeError};
  for (std::size_t d = 0; d < domain_count_; ++d) if (ecrt_domain_data(domains_[d]) == nullptr) return {mca::core::StatusCode::RuntimeError};
  activated_ = true;
  return state_.on_activated();
}

template <std::size_t S, std::size_t D, std::size_t E>
mca::core::Status EthercatIghAdapter<S, D, E>::exchange() noexcept {
  if (!activated_ || master_ == nullptr || state_.state() != EthercatMasterState::Active)
    return {mca::core::StatusCode::RuntimeError};
  if (ecrt_master_receive(master_) != 0) return {mca::core::StatusCode::RuntimeError};
  for (std::size_t d = 0; d < domain_count_; ++d) {
    if (ecrt_domain_process(domains_[d]) != 0) return {mca::core::StatusCode::RuntimeError};
    ec_domain_state_t s{};
    if (ecrt_domain_state(domains_[d], &s) != 0) return {mca::core::StatusCode::RuntimeError};
    observed_wkc_ = static_cast<std::uint16_t>(std::min(s.working_counter, 0xFFFFU));
    if (s.wc_state != EC_WC_COMPLETE) return {mca::core::StatusCode::RuntimeError};
  }
  for (std::size_t d = 0; d < domain_count_; ++d) if (ecrt_domain_queue(domains_[d]) != 0) return {mca::core::StatusCode::RuntimeError};
  if (ecrt_master_send(master_) != 0) return {mca::core::StatusCode::RuntimeError};
  ++cycle_sequence_;
  return {};
}

template <std::size_t S, std::size_t D, std::size_t E>
mca::core::Status EthercatIghAdapter<S, D, E>::shutdown() noexcept {
  if (master_ == nullptr) return {};
  if (activated_) { ecrt_master_deactivate(master_); activated_ = false; }
  ecrt_release_master(master_); master_ = nullptr; domain_count_ = 0U; binding_count_ = 0U;
  return {};
}

template class EthercatIghAdapter<16, 4, 128>;

}  // namespace mca::hardware
#endif
