#include "mca/hardware/ethercat_igh_adapter.hpp"

#if defined(MCA_ENABLE_IGH)
#include <algorithm>

#include "mca/hardware/ethercat_pdo_image.hpp"

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
  if (master_ == nullptr || state_.state() != EthercatMasterState::Discovered ||
      count == 0U || count > E || config_.cycle_time_ns == 0U)
    return {mca::core::StatusCode::InvalidArgument};

  // IgH has no public per-domain destroy operation. Once a domain or slave
  // configuration has been created, a later registration failure cannot be
  // rolled back in place. Release the inactive master and return to Offline so
  // callers cannot accidentally retry against partially configured objects.
  const auto fail_configuration = [this](mca::core::Status status) noexcept {
    if (activated_) {
      (void)ecrt_master_deactivate(master_);
      activated_ = false;
    }
    if (master_ != nullptr) {
      ecrt_release_master(master_);
      master_ = nullptr;
    }
    domains_.fill(nullptr);
    slave_configs_.fill(nullptr);
    domain_data_.fill(nullptr);
    domain_size_.fill(0U);
    offsets_.fill(0U);
    bit_positions_.fill(0U);
    domain_wkc_.fill(0U);
    domain_wc_complete_.fill(false);
    domain_count_ = 0U;
    binding_count_ = 0U;
    observed_wkc_ = 0U;
    cycle_sequence_ = 0U;
    runtime_health_ = {};
    (void)state_.on_shutdown();
    return status;
  };

  // Validate the complete caller description before making any irreversible
  // IgH registration calls. Topology order is not assumed to equal station
  // position; locate each slave by its explicit station address.
  for (std::size_t i = 0; i < count; ++i) {
    const auto& binding = bindings[i];
    if (binding.index == 0U || binding.bit_length == 0U ||
        binding.bit_length > 64U ||
        (binding.direction != EthercatPdoDirection::Input &&
         binding.direction != EthercatPdoDirection::Output))
      return {mca::core::StatusCode::InvalidArgument};

    const EthercatSlaveInfo* slave = nullptr;
    for (std::size_t s = 0; s < state_.topology().slave_count; ++s) {
      if (state_.topology().slaves[s].station_position ==
          binding.station_position) {
        slave = &state_.topology().slaves[s];
        break;
      }
    }
    if (slave == nullptr || slave->vendor_id != binding.vendor_id ||
        slave->product_code != binding.product_code)
      return {mca::core::StatusCode::ConstraintViolation};

    for (std::size_t j = 0; j < i; ++j) {
      const auto& prior = bindings[j];
      if (prior.station_position == binding.station_position &&
          prior.index == binding.index &&
          prior.subindex == binding.subindex)
        return {mca::core::StatusCode::InvalidArgument};
    }
  }

  std::array<std::uint16_t, D> ids{};
  std::array<std::size_t, E> topology_indices{};
  domain_count_ = 0U;
  binding_count_ = count;
  for (std::size_t i = 0; i < count; ++i) {
    const auto& binding = bindings[i];
    std::size_t domain = domain_count_;
    for (std::size_t d = 0; d < domain_count_; ++d) {
      if (ids[d] == binding.domain_id) {
        domain = d;
        break;
      }
    }
    if (domain == domain_count_) {
      if (domain_count_ >= D)
        return fail_configuration(
            {mca::core::StatusCode::ConstraintViolation});
      ids[domain_count_] = binding.domain_id;
      domains_[domain_count_] = ecrt_master_create_domain(master_);
      if (domains_[domain_count_] == nullptr)
        return fail_configuration(
            {mca::core::StatusCode::RuntimeError});
      ++domain_count_;
    }

    const auto& topology = state_.topology();
    std::size_t topology_index = topology.slave_count;
    for (std::size_t s = 0; s < topology.slave_count; ++s) {
      if (topology.slaves[s].station_position == binding.station_position) {
        topology_index = s;
        break;
      }
    }
    if (topology_index == topology.slave_count)
      return fail_configuration(
          {mca::core::StatusCode::ConstraintViolation});
    topology_indices[i] = topology_index;

    if (slave_configs_[topology_index] == nullptr) {
      slave_configs_[topology_index] = ecrt_master_slave_config(
          master_, 0U, binding.station_position, binding.vendor_id,
          binding.product_code);
    }
    if (slave_configs_[topology_index] == nullptr)
      return fail_configuration(
          {mca::core::StatusCode::RuntimeError});

    unsigned int bit = 0U;
    const int offset = ecrt_slave_config_reg_pdo_entry(
        slave_configs_[topology_index], binding.index,
        binding.subindex, domains_[domain], &bit);
    if (offset < 0)
      return fail_configuration(
          {mca::core::StatusCode::RuntimeError});
    offsets_[i] = static_cast<unsigned int>(offset);
    bit_positions_[i] = bit;
  }

  Layout layout{};
  layout.domain_count = domain_count_;
  layout.entry_count = count;
  for (std::size_t d = 0; d < domain_count_; ++d) {
    layout.domains[d].id = ids[d];
    layout.domains[d].cycle_time_ns = config_.cycle_time_ns;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const auto& binding = bindings[i];
    auto& entry = layout.entries[i];
    // EthercatPdoEntry::slave_position is a bounded topology slot, not the
    // EtherCAT station address. The latter may be sparse/non-zero-based.
    entry.slave_position = static_cast<std::uint16_t>(topology_indices[i]);
    entry.domain_id = binding.domain_id;
    entry.index = binding.index;
    entry.subindex = binding.subindex;
    entry.bit_length = binding.bit_length;
    entry.direction = binding.direction;
    entry.bit_offset = offsets_[i] * 8U + bit_positions_[i];
    for (std::size_t d = 0; d < domain_count_; ++d) {
      if (layout.domains[d].id != binding.domain_id) continue;
      const std::uint32_t bit_end =
          entry.bit_offset + static_cast<std::uint32_t>(binding.bit_length);
      if (binding.direction == EthercatPdoDirection::Input)
        layout.domains[d].input_bit_size =
            std::max(layout.domains[d].input_bit_size, bit_end);
      else
        layout.domains[d].output_bit_size =
            std::max(layout.domains[d].output_bit_size, bit_end);
      break;
    }
  }

  const auto status = state_.on_configured(layout);
  if (!status.ok()) return fail_configuration(status);
  return {};
}

template <std::size_t S, std::size_t D, std::size_t E>
std::size_t EthercatIghAdapter<S, D, E>::domain_index(std::uint16_t domain_id) const noexcept {
  const auto& layout = state_.layout();
  for (std::size_t d = 0; d < layout.domain_count; ++d)
    if (layout.domains[d].id == domain_id) return d;
  return D;
}

template <std::size_t S, std::size_t D, std::size_t E>
std::uint16_t EthercatIghAdapter<S, D, E>::domain_working_counter(
    std::uint16_t domain_id) const noexcept {
  const std::size_t index = domain_index(domain_id);
  return index < domain_count_ ? domain_wkc_[index] : 0U;
}

template <std::size_t S, std::size_t D, std::size_t E>
bool EthercatIghAdapter<S, D, E>::domain_working_counter_complete(
    std::uint16_t domain_id) const noexcept {
  const std::size_t index = domain_index(domain_id);
  return index < domain_count_ && domain_wc_complete_[index];
}

template <std::size_t S, std::size_t D, std::size_t E>
mca::core::Status EthercatIghAdapter<S, D, E>::configure_sdo_profile(
    const SdoProfile& profile) noexcept {
  if (master_ == nullptr || activated_ ||
      state_.state() != EthercatMasterState::Configured ||
      !profile.valid() || profile.slot_count > E || sdo_slot_count_ != 0U)
    return {mca::core::StatusCode::InvalidArgument};

  sdo_profile_ = profile;
  for (std::size_t i = 0; i < profile.slot_count; ++i) {
    const auto& slot = profile.slots[i];
    std::size_t topology_index = state_.topology().slave_count;
    for (std::size_t s = 0; s < state_.topology().slave_count; ++s) {
      if (state_.topology().slaves[s].station_position == slot.slave_position) {
        topology_index = s;
        break;
      }
    }
    if (topology_index >= state_.topology().slave_count ||
        slave_configs_[topology_index] == nullptr) {
      (void)shutdown();
      return {mca::core::StatusCode::ConstraintViolation};
    }

    auto* request = ecrt_slave_config_create_sdo_request(
        slave_configs_[topology_index], slot.index, slot.subindex,
        slot.transfer_capacity);
    if (request == nullptr ||
        ecrt_sdo_request_timeout(request, slot.timeout_ms) != 0) {
      (void)shutdown();
      return {mca::core::StatusCode::RuntimeError};
    }
    sdo_slots_[i].request = request;
    ++sdo_slot_count_;
  }
  return {};
}

template <std::size_t S, std::size_t D, std::size_t E>
mca::core::Status EthercatIghAdapter<S, D, E>::enqueue_sdo(
    const SdoRequest& request) noexcept {
  if (!activated_ || master_ == nullptr ||
      state_.state() != EthercatMasterState::Active ||
      !sdo_profile_.supports(request))
    return {mca::core::StatusCode::InvalidArgument};

  for (std::size_t i = 0; i < sdo_slot_count_; ++i) {
    if (sdo_slots_[i].occupied &&
        sdo_slots_[i].request_id == request.request_id)
      return {mca::core::StatusCode::RuntimeError};
  }

  for (std::size_t i = 0; i < sdo_slot_count_; ++i) {
    auto& slot = sdo_slots_[i];
    if (slot.occupied || slot.request == nullptr) continue;
    const auto& config = sdo_profile_.slots[i];
    if (config.slave_position != request.slave_position ||
        config.index != request.index || config.subindex != request.subindex)
      continue;
    // Check only the matching backend object. A failed prior submission may
    // leave one slot BUSY; it must not prevent dispatch to an unrelated slot.
    if (ecrt_sdo_request_state(slot.request) == EC_REQUEST_BUSY)
      continue;

    if (request.operation == EthercatSdoOperation::Write) {
      auto* data = ecrt_sdo_request_data(slot.request);
      if (data == nullptr) return {mca::core::StatusCode::RuntimeError};
      std::copy_n(request.payload.begin(), request.payload_size, data);
      if (ecrt_sdo_request_write(slot.request) != 0)
        return {mca::core::StatusCode::RuntimeError};
    } else {
      if (ecrt_sdo_request_read(slot.request) != 0)
        return {mca::core::StatusCode::RuntimeError};
    }
    slot.request_id = request.request_id;
    slot_deadlines_[i] = request.deadline_timestamp_ns;
    slot.operation = request.operation;
    slot.max_response_size = request.max_response_size;
    slot.timed_out = false;
    slot.occupied = true;
    return {};
  }
  return {mca::core::StatusCode::RuntimeError};
}

template <std::size_t S, std::size_t D, std::size_t E>
mca::core::Status EthercatIghAdapter<S, D, E>::poll_sdo(
    std::uint64_t request_id, std::uint64_t now_timestamp_ns,
    SdoResult& output) noexcept {
  if (!activated_ || request_id == 0U || now_timestamp_ns == 0U)
    return {mca::core::StatusCode::InvalidArgument};

  for (std::size_t i = 0; i < sdo_slot_count_; ++i) {
    auto& slot = sdo_slots_[i];
    if (!slot.occupied || slot.request_id != request_id) continue;

    SdoResult result{};
    result.request_id = request_id;
    const auto state = ecrt_sdo_request_state(slot.request);
    if (slot.timed_out) {
      if (state != EC_REQUEST_BUSY && state != EC_REQUEST_UNUSED) {
        slot.occupied = false;
        slot.request_id = 0U;
      }
      result.state = EthercatMailboxResultState::TimedOut;
      output = result;
      return {};
    }

    // IgH does not expose a completion timestamp. The adapter therefore uses
    // observation-time deadline semantics: a terminal result first observed
    // after the MCA deadline is reported TimedOut, not retroactively on-time.
    const std::uint64_t deadline = slot_deadlines_[i];
    if (now_timestamp_ns >= deadline) {
      slot.timed_out = true;
      if (state != EC_REQUEST_BUSY && state != EC_REQUEST_UNUSED) {
        slot.occupied = false;
        slot.request_id = 0U;
      }
      result.state = EthercatMailboxResultState::TimedOut;
      output = result;
      return {};
    }

    if (state == EC_REQUEST_BUSY || state == EC_REQUEST_UNUSED) {
      result.state = EthercatMailboxResultState::Pending;
      output = result;
      return {};
    }
    if (state == EC_REQUEST_ERROR) {
      result.state = EthercatMailboxResultState::Failed;
      slot.occupied = false;
      slot.request_id = 0U;
      output = result;
      return {};
    }
    if (state == EC_REQUEST_SUCCESS) {
      result.state = EthercatMailboxResultState::Completed;
      if (slot.operation == EthercatSdoOperation::Read) {
        const std::size_t size = ecrt_sdo_request_data_size(slot.request);
        const auto* data = ecrt_sdo_request_data(slot.request);
        if (size > slot.max_response_size || size > kMaxSdoBytes ||
            (size != 0U && data == nullptr)) {
          result.state = EthercatMailboxResultState::Failed;
        } else {
          if (size != 0U) std::copy_n(data, size, result.payload.begin());
          result.payload_size = size;
        }
      }
      slot.occupied = false;
      slot.request_id = 0U;
      output = result;
      return {};
    }
    result.state = EthercatMailboxResultState::Failed;
    slot.occupied = false;
    slot.request_id = 0U;
    output = result;
    return {};
  }
  return {mca::core::StatusCode::InvalidArgument};
}

template <std::size_t S, std::size_t D, std::size_t E>
mca::core::Status EthercatIghAdapter<S, D, E>::activate() noexcept {
  if (master_ == nullptr || state_.state() != EthercatMasterState::Configured)
    return {mca::core::StatusCode::RuntimeError};
  if (config_.send_interval_us != 0U && ecrt_master_set_send_interval(master_, config_.send_interval_us) != 0)
    return {mca::core::StatusCode::RuntimeError};
  if (ecrt_master_activate(master_) != 0) {
    // IgH may fail while finalizing one of several domains or while starting
    // its operation thread. The public API does not promise rollback to a
    // reusable configuration, so discard the master instead of retrying it.
    (void)shutdown();
    return {mca::core::StatusCode::RuntimeError};
  }
  for (std::size_t d = 0; d < domain_count_; ++d) {
    domain_data_[d] = ecrt_domain_data(domains_[d]);
    domain_size_[d] = ecrt_domain_size(domains_[d]);
    if (domain_data_[d] == nullptr || domain_size_[d] == 0U) {
      const int deactivate_result = ecrt_master_deactivate(master_);
      for (std::size_t i = 0; i < domain_count_; ++i) {
        domain_data_[i] = nullptr;
        domain_size_[i] = 0U;
      }
      // IgH deactivation destroys domains, slave configurations and SDO
      // request objects even when it succeeds. The configured backend cannot
      // be reused, so release the master and return to Offline in both cases.
      (void)deactivate_result;
      (void)shutdown();
      return {mca::core::StatusCode::RuntimeError};
    }
  }
  activated_ = true;
  const auto status = state_.on_activated();
  if (!status.ok()) {
    const int deactivate_result = ecrt_master_deactivate(master_);
    activated_ = false;
    for (std::size_t i = 0; i < domain_count_; ++i) {
      domain_data_[i] = nullptr;
      domain_size_[i] = 0U;
    }
    // A successful IgH deactivation also destroys the configuration
    // objects, so this master is never retryable after this rollback path.
    (void)deactivate_result;
    (void)shutdown();
  }
  return status;
}

template <std::size_t S, std::size_t D, std::size_t E>
mca::core::Status EthercatIghAdapter<S, D, E>::exchange() noexcept {
  if (!activated_ || master_ == nullptr || state_.state() != EthercatMasterState::Active)
    return {mca::core::StatusCode::RuntimeError};

  // Keep the cyclic datagram pipeline moving even when health diagnostics
  // or a domain report a fault. These IgH state APIs are realtime-safe and
  // expose link/master and per-configured-slave state independently of WKC.
  bool cycle_ok = (ecrt_master_receive(master_) == 0);

  ec_master_state_t master_state{};
  const int master_state_result = ecrt_master_state(master_, &master_state);
  runtime_health_.master_state_valid = (master_state_result == 0);
  runtime_health_.master_state_error = master_state_result;
  runtime_health_.link_up = false;
  runtime_health_.slaves_responding = 0U;
  runtime_health_.al_states = 0U;
  if (master_state_result == 0) {
    runtime_health_.link_up = (master_state.link_up != 0U);
    runtime_health_.slaves_responding = master_state.slaves_responding;
    runtime_health_.al_states = static_cast<std::uint8_t>(master_state.al_states);
  } else {
    cycle_ok = false;
  }

  runtime_health_.slave_count = state_.topology().slave_count;
  for (std::size_t i = 0; i < runtime_health_.slave_count; ++i) {
    auto& health = runtime_health_.slaves[i];
    health = {};
    health.station_position = state_.topology().slaves[i].station_position;
    health.configured = (slave_configs_[i] != nullptr);
    if (!health.configured) continue;

    ec_slave_config_state_t slave_state{};
    const int slave_state_result =
        ecrt_slave_config_state(slave_configs_[i], &slave_state);
    health.state_error = slave_state_result;
    health.state_valid = (slave_state_result == 0);
    if (slave_state_result == 0) {
      health.online = (slave_state.online != 0U);
      health.operational = (slave_state.operational != 0U);
      health.al_state = static_cast<std::uint8_t>(slave_state.al_state);
    } else {
      cycle_ok = false;
    }
  }

  std::uint32_t total_wkc = 0U;
  bool all_domains_complete = true;

  for (std::size_t d = 0; d < domain_count_; ++d) {
    if (ecrt_domain_process(domains_[d]) != 0) cycle_ok = false;
    ec_domain_state_t s{};
    if (ecrt_domain_state(domains_[d], &s) != 0) {
      cycle_ok = false;
      domain_wkc_[d] = 0U;
      domain_wc_complete_[d] = false;
      all_domains_complete = false;
      continue;
    }
    domain_wkc_[d] = static_cast<std::uint16_t>(std::min(s.working_counter, 0xFFFFU));
    domain_wc_complete_[d] = (s.wc_state == EC_WC_COMPLETE);
    total_wkc += domain_wkc_[d];
    if (!domain_wc_complete_[d]) all_domains_complete = false;
  }

  observed_wkc_ = static_cast<std::uint16_t>(std::min(total_wkc, 0xFFFFU));
  for (std::size_t d = 0; d < domain_count_; ++d)
    if (ecrt_domain_queue(domains_[d]) != 0) cycle_ok = false;
  if (ecrt_master_send(master_) != 0) cycle_ok = false;
  else ++cycle_sequence_;
  // Health sequence counts diagnostic samples, independently of successful
  // PDO sends, so a failed send cannot make a fresh snapshot look stale.
  runtime_health_.sequence = runtime_health_.sequence == UINT64_MAX
                                 ? 1U
                                 : runtime_health_.sequence + 1U;

  if (!all_domains_complete || !cycle_ok)
    return {mca::core::StatusCode::RuntimeError};
  return {};
}

template <std::size_t S, std::size_t D, std::size_t E>
mca::core::Status EthercatIghAdapter<S, D, E>::read_pdo(
    std::size_t binding_index, std::uint64_t& value) const noexcept {
  if (!activated_ || state_.state() != EthercatMasterState::Active || binding_index >= binding_count_)
    return {mca::core::StatusCode::RuntimeError};
  const auto& entry = state_.layout().entries[binding_index];
  if (entry.direction != EthercatPdoDirection::Input)
    return {mca::core::StatusCode::InvalidArgument};
  const std::size_t di = domain_index(entry.domain_id);
  if (di >= domain_count_ || domain_data_[di] == nullptr)
    return {mca::core::StatusCode::RuntimeError};
  if (!ethercat_read_pdo_bits(domain_data_[di], domain_size_[di], entry.bit_offset, entry.bit_length, value))
    return {mca::core::StatusCode::ConstraintViolation};
  return {};
}

template <std::size_t S, std::size_t D, std::size_t E>
mca::core::Status EthercatIghAdapter<S, D, E>::write_pdo(
    std::size_t binding_index, std::uint64_t value) noexcept {
  if (!activated_ || state_.state() != EthercatMasterState::Active || binding_index >= binding_count_)
    return {mca::core::StatusCode::RuntimeError};
  const auto& entry = state_.layout().entries[binding_index];
  if (entry.direction != EthercatPdoDirection::Output)
    return {mca::core::StatusCode::InvalidArgument};
  const std::size_t di = domain_index(entry.domain_id);
  if (di >= domain_count_ || domain_data_[di] == nullptr)
    return {mca::core::StatusCode::RuntimeError};
  if (!ethercat_write_pdo_bits(domain_data_[di], domain_size_[di], entry.bit_offset, entry.bit_length, value))
    return {mca::core::StatusCode::ConstraintViolation};
  return {};
}

template <std::size_t S, std::size_t D, std::size_t E>
mca::core::Status EthercatIghAdapter<S, D, E>::shutdown() noexcept {
  // Keep cleanup idempotent even if the master pointer was already cleared by
  // a partial initialization/rollback path. Adapter-owned handles and SDO
  // bookkeeping must never survive a logical shutdown.
  int deactivate_result = 0;
  if (master_ != nullptr) {
    if (activated_) {
      deactivate_result = ecrt_master_deactivate(master_);
      activated_ = false;
      // The state machine rejects shutdown while Active. Once the IgH master
      // has been deactivated (and will be released below), mirror that
      // lifecycle transition before asking the state machine to go Offline.
      (void)state_.on_deactivated();
    }
    ecrt_release_master(master_);
    master_ = nullptr;
  }
  domains_.fill(nullptr);
  slave_configs_.fill(nullptr);
  domain_data_.fill(nullptr);
  domain_size_.fill(0U);
  offsets_.fill(0U);
  bit_positions_.fill(0U);
  domain_count_ = 0U;
  binding_count_ = 0U;
  sdo_slots_.fill({});
  slot_deadlines_.fill(0U);
  sdo_slot_count_ = 0U;
  sdo_profile_ = {};
  observed_wkc_ = 0U;
  cycle_sequence_ = 0U;
  runtime_health_ = {};
  domain_wkc_.fill(0U);
  domain_wc_complete_.fill(false);
  const auto shutdown_status = state_.on_shutdown();
  if (deactivate_result != 0)
    return {mca::core::StatusCode::RuntimeError};
  return shutdown_status;
}

template class EthercatIghAdapter<16, 4, 128>;

}  // namespace mca::hardware
#endif
