#pragma once
#include <cstddef>
#include <cstdint>
#include "ecrt.h"

namespace fake_igh {
void reset() noexcept;
void fail_create_at(std::size_t attempt) noexcept;
void fail_timeout_at(std::size_t request_index) noexcept;
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
