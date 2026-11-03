#include "fake_igh_api.hpp"
#include <array>
#include <algorithm>
#include <cstring>

struct ec_master {};
struct ec_domain { std::array<std::uint8_t, 8> data{}; };
struct ec_slave_config {};
struct ec_sdo_request {
  ec_request_state_t state{EC_REQUEST_UNUSED};
  std::array<std::uint8_t, 256> data{};
  std::array<std::uint8_t, 256> relocated_data{};
  bool relocated{false};
  std::size_t size{0U};
  std::size_t capacity{0U};
  std::size_t reads{0U};
  std::size_t writes{0U};
};

namespace {
struct CapturedEntry { std::uint16_t index{0U}; std::uint8_t width{0U}; };
struct CapturedPdo {
  std::uint16_t index{0U};
  std::size_t entry_count{0U};
  std::array<CapturedEntry, 256U> entries{};
};
struct CapturedSync {
  std::uint8_t index{0U};
  ec_direction_t direction{EC_DIR_INVALID};
  std::size_t pdo_count{0U};
  std::array<CapturedPdo, 256U> pdos{};
};
ec_master master;
ec_domain domain;
ec_slave_config slave;
std::array<ec_sdo_request, 8> requests{};
std::size_t created = 0U;
std::size_t create_attempts = 0U;
std::size_t create_failure = static_cast<std::size_t>(-1);
std::size_t timeout_failure = static_cast<std::size_t>(-1);
std::size_t releases = 0U;
std::size_t deactivations = 0U;
bool activate_failure = false;
bool pdo_configuration_failure = false;
bool registration_seen = false;
bool pdo_order_valid = true;
std::size_t pdo_configurations = 0U;
std::size_t pdo_registrations = 0U;
std::size_t captured_syncs = 0U;
std::array<CapturedSync, 256U> captured{};
bool domain_data_failure = false;
bool zero_size = false;
bool deactivate_failure = false;
bool master_state_failure = false;
bool slave_state_failure = false;
bool master_link_up = true;
unsigned int master_responding = 1U;
unsigned int master_al_states = EC_AL_STATE_OP;
bool slave_online = true;
bool slave_operational = true;
std::uint8_t slave_al_state = EC_AL_STATE_OP;
std::size_t queues = 0U;
std::size_t sends = 0U;
std::size_t activations = 0U;
}

namespace fake_igh {
void on_release() noexcept { ++releases; created = 0U; }
void on_deactivate() noexcept { ++deactivations; }
void reset() noexcept {
  created = 0U;
  create_attempts = 0U;
  create_failure = static_cast<std::size_t>(-1);
  timeout_failure = static_cast<std::size_t>(-1);
  releases = 0U;
  deactivations = 0U;
  activate_failure = false; pdo_configuration_failure = false;
  registration_seen = false; pdo_order_valid = true;
  pdo_configurations = 0U; pdo_registrations = 0U;
  captured_syncs = 0U; captured = {};
  domain_data_failure = false;
  zero_size = false; deactivate_failure = false;
  master_state_failure = false; slave_state_failure = false;
  master_link_up = true; master_responding = 1U; master_al_states = EC_AL_STATE_OP;
  slave_online = true; slave_operational = true; slave_al_state = EC_AL_STATE_OP;
  queues = 0U; sends = 0U; activations = 0U;
  domain = {};
  requests = {};
}
std::size_t request_count() noexcept { return created; }
std::size_t live_request_count() noexcept { return created; }
void fail_create_at(std::size_t attempt) noexcept { create_failure = attempt; }
void fail_timeout_at(std::size_t index) noexcept { timeout_failure = index; }
void fail_activate(bool enabled) noexcept { activate_failure = enabled; }
void fail_pdo_configuration(bool enabled) noexcept { pdo_configuration_failure = enabled; }
std::size_t pdo_configuration_calls() noexcept { return pdo_configurations; }
std::size_t pdo_registration_calls() noexcept { return pdo_registrations; }
bool pdo_configuration_preceded_registration() noexcept { return pdo_order_valid; }
std::size_t configured_sync_count() noexcept { return captured_syncs; }
std::uint8_t configured_sync_index(std::size_t i) noexcept {
  return i < captured_syncs ? captured[i].index : 0U;
}
ec_direction_t configured_sync_direction(std::size_t i) noexcept {
  return i < captured_syncs ? captured[i].direction : EC_DIR_INVALID;
}
std::size_t configured_pdo_count(std::size_t i) noexcept {
  return i < captured_syncs ? captured[i].pdo_count : 0U;
}
std::uint16_t configured_pdo_index(std::size_t i, std::size_t p) noexcept {
  return i < captured_syncs && p < captured[i].pdo_count ? captured[i].pdos[p].index : 0U;
}
std::size_t configured_entry_count(std::size_t i, std::size_t p) noexcept {
  return i < captured_syncs && p < captured[i].pdo_count ? captured[i].pdos[p].entry_count : 0U;
}
std::uint16_t configured_entry_index(std::size_t i, std::size_t p, std::size_t e) noexcept {
  return i < captured_syncs && p < captured[i].pdo_count &&
      e < captured[i].pdos[p].entry_count ? captured[i].pdos[p].entries[e].index : 0U;
}
std::uint8_t configured_entry_width(std::size_t i, std::size_t p, std::size_t e) noexcept {
  return i < captured_syncs && p < captured[i].pdo_count &&
      e < captured[i].pdos[p].entry_count ? captured[i].pdos[p].entries[e].width : 0U;
}
void fail_domain_data(bool enabled) noexcept { domain_data_failure = enabled; }
void zero_domain_size(bool enabled) noexcept { zero_size = enabled; }
void fail_deactivate(bool enabled) noexcept { deactivate_failure = enabled; }
void set_master_state(bool link_up, unsigned int responding, unsigned int al_states) noexcept {
  master_link_up = link_up; master_responding = responding; master_al_states = al_states;
}
void set_slave_state(bool online, bool operational, std::uint8_t al_state) noexcept {
  slave_online = online; slave_operational = operational; slave_al_state = al_state;
}
void fail_master_state(bool enabled) noexcept { master_state_failure = enabled; }
void fail_slave_state(bool enabled) noexcept { slave_state_failure = enabled; }
std::size_t queue_calls() noexcept { return queues; }
std::size_t send_calls() noexcept { return sends; }
std::size_t activate_calls() noexcept { return activations; }
void record_activate() noexcept { ++activations; }
bool activation_fails() noexcept { return activate_failure; }
bool domain_data_fails() noexcept { return domain_data_failure; }
bool domain_size_is_zero() noexcept { return zero_size; }
bool deactivation_fails() noexcept { return deactivate_failure; }

std::size_t release_calls() noexcept { return releases; }
std::size_t deactivate_calls() noexcept { return deactivations; }
ec_sdo_request_t* request(std::size_t index) noexcept {
  return index < created ? &requests[index] : nullptr;
}
void complete(std::size_t index, const std::uint8_t* data, std::size_t size) noexcept {
  if (index >= created) return;
  auto& r = requests[index];
  r.relocated = false;
  r.size = std::min(size, r.data.size());
  if (data != nullptr && r.size != 0U) std::copy_n(data, r.size, r.data.begin());
  r.state = EC_REQUEST_SUCCESS;
}
void relocate_and_complete(std::size_t index, const std::uint8_t* data, std::size_t size) noexcept {
  if (index >= created) return;
  auto& r = requests[index];
  r.relocated = true;
  r.size = std::min(size, r.relocated_data.size());
  if (data != nullptr && r.size != 0U)
    std::copy_n(data, r.size, r.relocated_data.begin());
  r.state = EC_REQUEST_SUCCESS;
}
void fail(std::size_t index) noexcept {
  if (index < created) requests[index].state = EC_REQUEST_ERROR;
}
void set_busy(std::size_t index) noexcept {
  if (index < created) requests[index].state = EC_REQUEST_BUSY;
}
std::size_t read_calls(std::size_t index) noexcept {
  return index < created ? requests[index].reads : 0U;
}
std::size_t write_calls(std::size_t index) noexcept {
  return index < created ? requests[index].writes : 0U;
}
}

extern "C" {
ec_master_t* ecrt_request_master(unsigned int) { return &master; }
void ecrt_release_master(ec_master_t*) { fake_igh::on_release(); }
int ecrt_master(ec_master_t*, ec_master_info_t* info) {
  info->slave_count = 1U; return 0;
}
int ecrt_master_state(const ec_master_t*, ec_master_state_t* state) {
  if (master_state_failure) return -1;
  state->slaves_responding = master_responding;
  state->al_states = master_al_states;
  state->link_up = master_link_up ? 1U : 0U;
  return 0;
}
int ecrt_slave_config_state(const ec_slave_config_t*, ec_slave_config_state_t* state) {
  if (slave_state_failure) return -1;
  state->online = slave_online ? 1U : 0U;
  state->operational = slave_operational ? 1U : 0U;
  state->al_state = slave_al_state;
  return 0;
}
int ecrt_master_get_slave(ec_master_t*, uint16_t position, ec_slave_info_t* info) {
  if (position != 0U) return -1;
  *info = {}; info->position = 0U; info->vendor_id = 0x2U;
  info->product_code = 0x1234U; info->revision_number = 1U;
  info->serial_number = 7U; info->al_state = EC_AL_STATE_PREOP; return 0;
}
ec_domain_t* ecrt_master_create_domain(ec_master_t*) { return &domain; }
ec_slave_config_t* ecrt_master_slave_config(ec_master_t*, uint16_t, uint16_t position,
                                             uint32_t vendor, uint32_t product) {
  return position == 0U && vendor == 0x2U && product == 0x1234U ? &slave : nullptr;
}
int ecrt_slave_config_pdos(ec_slave_config_t*, unsigned int count,
                            const ec_sync_info_t* syncs) {
  ++pdo_configurations;
  if (registration_seen) pdo_order_valid = false;
  if (pdo_configuration_failure || syncs == nullptr || count > captured.size())
    return -1;
  captured_syncs = count;
  for (std::size_t i = 0; i < captured_syncs; ++i) {
    captured[i].index = syncs[i].index;
    captured[i].direction = syncs[i].dir;
    captured[i].pdo_count = syncs[i].n_pdos;
    if (captured[i].pdo_count > captured[i].pdos.size()) return -1;
    for (std::size_t p = 0; p < captured[i].pdo_count; ++p) {
      const auto& source = syncs[i].pdos[p];
      auto& target = captured[i].pdos[p];
      target.index = source.index;
      target.entry_count = source.n_entries;
      if (target.entry_count > target.entries.size()) return -1;
      for (std::size_t e = 0; e < target.entry_count; ++e) {
        target.entries[e].index = source.entries[e].index;
        target.entries[e].width = source.entries[e].bit_length;
      }
    }
  }
  return 0;
}
int ecrt_master_get_sync_manager(ec_master_t*, uint16_t station,
                                uint8_t sync_index, ec_sync_info_t* sync) {
  if (station != 0U || sync == nullptr) return -1;
  *sync = {};
  if (sync_index == 2U) {
    sync->index = 2U; sync->dir = EC_DIR_INPUT; sync->n_pdos = 1U;
    return 0;
  }
  if (sync_index == 3U) {
    sync->index = 3U; sync->dir = EC_DIR_OUTPUT; sync->n_pdos = 1U;
    return 0;
  }
  return -1;
}
int ecrt_master_get_pdo(ec_master_t*, uint16_t station, uint8_t sync_index,
                        uint16_t pdo_pos, ec_pdo_info_t* pdo) {
  if (station != 5U || pdo_pos != 0U || pdo == nullptr) return -1;
  *pdo = {};
  if (sync_index == 2U) { pdo->index = 0x1A00U; pdo->n_entries = 2U; return 0; }
  if (sync_index == 3U) { pdo->index = 0x1600U; pdo->n_entries = 1U; return 0; }
  return -1;
}
int ecrt_master_get_pdo_entry(ec_master_t*, uint16_t station, uint8_t sync_index,
                              uint16_t pdo_pos, uint16_t entry_pos,
                              ec_pdo_entry_info_t* entry) {
  if (station != 5U || pdo_pos != 0U || entry == nullptr) return -1;
  if (sync_index == 2U && entry_pos == 0U)
    *entry = {0x6041U, 0U, 16U};
  else if (sync_index == 2U && entry_pos == 1U)
    *entry = {0x6064U, 0U, 32U};
  else if (sync_index == 3U && entry_pos == 0U)
    *entry = {0x6040U, 0U, 16U};
  else
    return -1;
  return 0;
}
int ecrt_slave_config_reg_pdo_entry(ec_slave_config_t*, uint16_t, uint8_t,
                                     ec_domain_t*, unsigned int* bit) {
  registration_seen = true;
  ++pdo_registrations;
  *bit = 0U; return 0;
}
ec_sdo_request_t* ecrt_slave_config_create_sdo_request(ec_slave_config_t*, uint16_t,
                                                        uint8_t, size_t capacity) {
  const std::size_t attempt = create_attempts++;
  if (attempt == create_failure || created >= requests.size()) return nullptr;
  auto& r = requests[created++]; r.capacity = capacity; r.size = capacity; return &r;
}
int ecrt_sdo_request_timeout(ec_sdo_request_t* r, uint32_t timeout) {
  const auto index = static_cast<std::size_t>(r - requests.data());
  return timeout == 0U || index == timeout_failure ? -1 : 0;
}
uint8_t* ecrt_sdo_request_data(ec_sdo_request_t* r) {
  return r->relocated ? r->relocated_data.data() : r->data.data();
}
size_t ecrt_sdo_request_data_size(const ec_sdo_request_t* r) { return r->size; }
int ecrt_sdo_request_read(ec_sdo_request_t* r) {
  if (r->state == EC_REQUEST_BUSY) return -1;
  ++r->reads; r->state = EC_REQUEST_BUSY; return 0;
}
int ecrt_sdo_request_write(ec_sdo_request_t* r) {
  if (r->state == EC_REQUEST_BUSY) return -1;
  ++r->writes; r->state = EC_REQUEST_BUSY; return 0;
}
ec_request_state_t ecrt_sdo_request_state(const ec_sdo_request_t* r) { return r->state; }
int ecrt_master_set_send_interval(ec_master_t*, size_t) { return 0; }
int ecrt_master_activate(ec_master_t*) { fake_igh::record_activate(); return fake_igh::activation_fails() ? -1 : 0; }
int ecrt_master_deactivate(ec_master_t*) { fake_igh::on_deactivate(); return fake_igh::deactivation_fails() ? -1 : 0; }
uint8_t* ecrt_domain_data(ec_domain_t* d) { return fake_igh::domain_data_fails() ? nullptr : d->data.data(); }
size_t ecrt_domain_size(const ec_domain_t* d) { return fake_igh::domain_size_is_zero() ? 0U : d->data.size(); }
int ecrt_master_receive(ec_master_t*) { return 0; }
int ecrt_domain_process(ec_domain_t*) { return 0; }
int ecrt_domain_state(const ec_domain_t*, ec_domain_state_t* s) {
  s->working_counter = 1U; s->wc_state = EC_WC_COMPLETE; return 0;
}
int ecrt_domain_queue(ec_domain_t*) { ++queues; return 0; }
int ecrt_master_send(ec_master_t*) { ++sends; return 0; }
}
