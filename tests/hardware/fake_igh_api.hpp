#pragma once
#include <cstddef>
#include <cstdint>
#include "ecrt.h"

namespace fake_igh {
void reset() noexcept;
void fail_create_at(std::size_t attempt) noexcept;
void fail_timeout_at(std::size_t request_index) noexcept;
void fail_activate(bool enabled) noexcept;
void fail_pdo_configuration(bool enabled) noexcept;
std::size_t pdo_configuration_calls() noexcept;
std::size_t pdo_registration_calls() noexcept;
bool pdo_configuration_preceded_registration() noexcept;
std::size_t configured_sync_count() noexcept;
std::uint8_t configured_sync_index(std::size_t sync) noexcept;
ec_direction_t configured_sync_direction(std::size_t sync) noexcept;
std::size_t configured_pdo_count(std::size_t sync) noexcept;
std::uint16_t configured_pdo_index(std::size_t sync, std::size_t pdo) noexcept;
std::size_t configured_entry_count(std::size_t sync, std::size_t pdo) noexcept;
std::uint16_t configured_entry_index(std::size_t sync, std::size_t pdo, std::size_t entry) noexcept;
std::uint8_t configured_entry_width(std::size_t sync, std::size_t pdo, std::size_t entry) noexcept;
void fail_domain_data(bool enabled) noexcept;
void zero_domain_size(bool enabled) noexcept;
void fail_deactivate(bool enabled) noexcept;
void set_master_state(bool link_up, unsigned int slaves_responding, unsigned int al_states) noexcept;
void set_slave_state(bool online, bool operational, std::uint8_t al_state) noexcept;
void set_slave_count(std::size_t count) noexcept;
void set_slave_state_at(std::size_t index, bool online, bool operational,
                        std::uint8_t al_state) noexcept;
void set_domain_working_counter_complete_at(std::size_t index,
                                             bool complete) noexcept;
void set_domain_working_counter_complete(bool complete) noexcept;
void fail_master_state(bool enabled) noexcept;
void fail_slave_state(bool enabled) noexcept;
void fail_receive(bool enabled) noexcept;
void fail_domain_process(bool enabled) noexcept;
void fail_domain_queue(bool enabled) noexcept;
void fail_send(bool enabled) noexcept;
std::size_t queue_calls() noexcept;
std::size_t send_calls() noexcept;
std::size_t activate_calls() noexcept;
void record_activate() noexcept;
bool activation_fails() noexcept;
bool domain_data_fails() noexcept;
bool domain_size_is_zero() noexcept;
bool deactivation_fails() noexcept;
std::size_t release_calls() noexcept;
std::size_t deactivate_calls() noexcept;
std::size_t request_count() noexcept;
std::size_t live_request_count() noexcept;
ec_sdo_request_t* request(std::size_t index) noexcept;
void complete(std::size_t index, const std::uint8_t* data, std::size_t size) noexcept;
void relocate_and_complete(std::size_t index, const std::uint8_t* data, std::size_t size) noexcept;
void fail(std::size_t index) noexcept;
void set_busy(std::size_t index) noexcept;
std::size_t read_calls(std::size_t index) noexcept;
std::size_t write_calls(std::size_t index) noexcept;
}
