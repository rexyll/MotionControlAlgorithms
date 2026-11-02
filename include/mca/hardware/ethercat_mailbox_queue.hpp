#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "mca/hardware/ethercat_master.hpp"

namespace mca::hardware {

// Deterministic bounded mailbox lifecycle primitive. The adapter supplies a
// monotonic timestamp; no clock, allocation, lock, or blocking operation is
// hidden in this type. One slot remains occupied until retire() is called.
template <std::size_t QueueCapacity, std::size_t MaxBytes>
class EthercatMailboxQueue {
 public:
  static_assert(QueueCapacity > 0U, "EtherCAT mailbox queue must have capacity");
  using Request = EthercatSdoRequest<MaxBytes>;
  using Result = EthercatSdoResult<MaxBytes>;

  core::Status enqueue(const Request& request,
                       std::uint64_t now_ns) noexcept {
    if (!request.valid() || now_ns == 0U)
      return {core::StatusCode::InvalidArgument};
    if (request.deadline_timestamp_ns <= now_ns)
      return {core::StatusCode::InvalidArgument};
    for (const auto& slot : slots_) {
      if (slot.occupied && slot.request.request_id == request.request_id)
        return {core::StatusCode::RuntimeError};
    }
    for (auto& slot : slots_) {
      if (slot.occupied) continue;
      slot.request = request;
      slot.result = {};
      slot.result.request_id = request.request_id;
      slot.result.state = EthercatMailboxResultState::Pending;
      slot.occupied = true;
      return {};
    }
    return {core::StatusCode::RuntimeError};
  }

  // Poll is non-blocking. Deadline equality is expired. The terminal timeout
  // remains queryable until explicitly retired by the adapter/client.
  core::Status poll(std::uint64_t request_id, std::uint64_t now_ns,
                    Result& output) noexcept {
    if (request_id == 0U || now_ns == 0U)
      return {core::StatusCode::InvalidArgument};
    auto* slot = find(request_id);
    if (slot == nullptr) return {core::StatusCode::InvalidArgument};
    if (slot->result.state == EthercatMailboxResultState::Pending &&
        now_ns >= slot->request.deadline_timestamp_ns) {
      slot->result.state = EthercatMailboxResultState::TimedOut;
    }
    output = slot->result;
    return {};
  }

  // A backend may complete only a live, pending request, and never after its
  // deadline. Timed-out requests cannot be resurrected by a late response.
  core::Status complete(std::uint64_t request_id, std::uint64_t now_ns,
                        const Result& result) noexcept {
    if (request_id == 0U || now_ns == 0U || !result.valid() ||
        result.request_id != request_id ||
        result.state == EthercatMailboxResultState::Pending ||
        result.state == EthercatMailboxResultState::TimedOut)
      return {core::StatusCode::InvalidArgument};
    auto* slot = find(request_id);
    if (slot == nullptr) return {core::StatusCode::InvalidArgument};
    if (slot->result.state != EthercatMailboxResultState::Pending ||
        now_ns >= slot->request.deadline_timestamp_ns)
      return {core::StatusCode::RuntimeError};
    if ((slot->request.operation == EthercatSdoOperation::Read &&
         result.payload_size > slot->request.max_response_size) ||
        (slot->request.operation == EthercatSdoOperation::Write &&
         result.payload_size != 0U))
      return {core::StatusCode::InvalidArgument};
    slot->result = result;
    return {};
  }

  // Retiring is explicit so repeated polls can observe a terminal result.
  core::Status retire(std::uint64_t request_id) noexcept {
    if (request_id == 0U) return {core::StatusCode::InvalidArgument};
    auto* slot = find(request_id);
    if (slot == nullptr) return {core::StatusCode::InvalidArgument};
    if (slot->result.state == EthercatMailboxResultState::Pending)
      return {core::StatusCode::RuntimeError};
    slot->occupied = false;
    return {};
  }

 private:
  struct Slot {
    Request request{};
    Result result{};
    bool occupied{false};
  };

  Slot* find(std::uint64_t request_id) noexcept {
    for (auto& slot : slots_) {
      if (slot.occupied && slot.request.request_id == request_id)
        return &slot;
    }
    return nullptr;
  }

  std::array<Slot, QueueCapacity> slots_{};
};

}  // namespace mca::hardware
