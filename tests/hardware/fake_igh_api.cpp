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
  std::size_t size{0U};
  std::size_t capacity{0U};
  std::size_t reads{0U};
  std::size_t writes{0U};
};

namespace {
ec_master master;
ec_domain domain;
ec_slave_config slave;
std::array<ec_sdo_request, 8> requests{};
std::size_t created = 0U;
}

namespace fake_igh {
void reset() noexcept {
  created = 0U;
  domain = {};
  requests = {};
}
std::size_t request_count() noexcept { return created; }
ec_sdo_request_t* request(std::size_t index) noexcept {
  return index < created ? &requests[index] : nullptr;
}
void complete(std::size_t index, const std::uint8_t* data, std::size_t size) noexcept {
  if (index >= created) return;
  auto& r = requests[index];
  r.size = std::min(size, r.capacity);
  if (data != nullptr && r.size != 0U) std::copy_n(data, r.size, r.data.begin());
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
void ecrt_release_master(ec_master_t*) {}
int ecrt_master(ec_master_t*, ec_master_info_t* info) {
  info->slave_count = 1U; return 0;
}
int ecrt_master_get_slave(ec_master_t*, uint16_t position, ec_slave_info_t* info) {
  if (position != 0U) return -1;
  *info = {}; info->position = 5U; info->vendor_id = 0x2U;
  info->product_code = 0x1234U; info->revision_number = 1U;
  info->serial_number = 7U; info->al_state = EC_AL_STATE_PREOP; return 0;
}
ec_domain_t* ecrt_master_create_domain(ec_master_t*) { return &domain; }
ec_slave_config_t* ecrt_master_slave_config(ec_master_t*, uint16_t, uint16_t position,
                                             uint32_t vendor, uint32_t product) {
  return position == 5U && vendor == 0x2U && product == 0x1234U ? &slave : nullptr;
}
int ecrt_slave_config_reg_pdo_entry(ec_slave_config_t*, uint16_t, uint8_t,
                                     ec_domain_t*, unsigned int* bit) {
  *bit = 0U; return 0;
}
ec_sdo_request_t* ecrt_slave_config_create_sdo_request(ec_slave_config_t*, uint16_t,
                                                        uint8_t, size_t capacity) {
  if (created >= requests.size()) return nullptr;
  auto& r = requests[created++]; r.capacity = capacity; return &r;
}
int ecrt_sdo_request_timeout(ec_sdo_request_t*, uint32_t timeout) { return timeout == 0U ? -1 : 0; }
uint8_t* ecrt_sdo_request_data(ec_sdo_request_t* r) { return r->data.data(); }
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
int ecrt_master_activate(ec_master_t*) { return 0; }
int ecrt_master_deactivate(ec_master_t*) { return 0; }
uint8_t* ecrt_domain_data(ec_domain_t* d) { return d->data.data(); }
size_t ecrt_domain_size(const ec_domain_t* d) { return d->data.size(); }
int ecrt_master_receive(ec_master_t*) { return 0; }
int ecrt_domain_process(ec_domain_t*) { return 0; }
int ecrt_domain_state(const ec_domain_t*, ec_domain_state_t* s) {
  s->working_counter = 1U; s->wc_state = EC_WC_COMPLETE; return 0;
}
int ecrt_domain_queue(ec_domain_t*) { return 0; }
int ecrt_master_send(ec_master_t*) { return 0; }
}
