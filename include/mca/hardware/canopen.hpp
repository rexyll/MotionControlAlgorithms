#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "mca/core/types.hpp"

namespace mca::hardware {

// CANopen uses 11-bit identifiers for the base communication profile. Extended
// frames are rejected here rather than silently interpreted as CANopen frames.
struct CanopenCanFrame {
  std::uint16_t cob_id{0};
  std::uint8_t dlc{0};
  std::array<std::uint8_t, 8> data{};

  [[nodiscard]] constexpr bool valid() const noexcept {
    return cob_id <= 0x07FFU && dlc <= 8U;
  }
};

enum class CanopenNmtState : std::uint8_t {
  Initialising = 0x00U,
  Stopped = 0x04U,
  Operational = 0x05U,
  PreOperational = 0x7FU
};

enum class CanopenTransportState : std::uint8_t {
  Offline,
  Online,
  BusOff
};

class ICanopenTransport {
 public:
  virtual ~ICanopenTransport() = default;
  virtual core::Status send(const CanopenCanFrame& frame) noexcept = 0;
  // Non-blocking receive reports an empty queue separately from transport
  // errors; callers never need to overload a status code for "no frame".
  virtual core::Status receive(CanopenCanFrame& frame,
                               bool& has_frame) noexcept = 0;
  [[nodiscard]] virtual CanopenTransportState state() const noexcept = 0;
};

struct CanopenEmcy {
  std::uint16_t error_code{0};
  std::uint8_t error_register{0};
  std::array<std::uint8_t, 5> manufacturer_data{};
};

constexpr bool decode_canopen_emcy(const CanopenCanFrame& frame,
                                   std::uint8_t node_id,
                                   CanopenEmcy& output) noexcept {
  if (!frame.valid() || node_id == 0U || node_id > 127U ||
      frame.cob_id != static_cast<std::uint16_t>(0x080U + node_id) ||
      frame.dlc != 8U) {
    return false;
  }
  CanopenEmcy decoded;
  decoded.error_code = static_cast<std::uint16_t>(frame.data[0]) |
                       (static_cast<std::uint16_t>(frame.data[1]) << 8U);
  decoded.error_register = frame.data[2];
  for (std::size_t i = 0; i < decoded.manufacturer_data.size(); ++i) {
    decoded.manufacturer_data[i] = frame.data[i + 3U];
  }
  output = decoded;
  return true;
}

class CanopenHeartbeatMonitor {
 public:
  constexpr explicit CanopenHeartbeatMonitor(std::uint8_t node_id = 1U,
                                              std::uint32_t timeout_ms = 1000U)
      : node_id_(node_id), timeout_ms_(timeout_ms) {}

  [[nodiscard]] constexpr bool valid() const noexcept {
    return node_id_ > 0U && node_id_ <= 127U && timeout_ms_ > 0U;
  }

  bool observe(const CanopenCanFrame& frame, std::uint64_t now_ms) noexcept {
    if (!valid() || !frame.valid() ||
        frame.cob_id != static_cast<std::uint16_t>(0x700U + node_id_) ||
        frame.dlc != 1U ||\n        (frame.data[0] != 0x00U && frame.data[0] != 0x04U &&\n         frame.data[0] != 0x05U && frame.data[0] != 0x7FU)) {
      return false;
    }
    last_state_ = frame.data[0];
    last_seen_ms_ = now_ms;
    seen_ = true;
    timed_out_ = false;
    return true;
  }

  [[nodiscard]] bool poll(std::uint64_t now_ms) noexcept {
    if (!seen_ || now_ms < last_seen_ms_) return false;
    if (now_ms - last_seen_ms_ >= timeout_ms_) timed_out_ = true;
    return timed_out_;
  }

  [[nodiscard]] bool seen() const noexcept { return seen_; }
  [[nodiscard]] bool timed_out() const noexcept { return timed_out_; }
  [[nodiscard]] std::uint8_t last_state() const noexcept { return last_state_; }

 private:
  std::uint8_t node_id_{1U};
  std::uint32_t timeout_ms_{1000U};
  std::uint64_t last_seen_ms_{0U};
  std::uint8_t last_state_{0U};
  bool seen_{false};
  bool timed_out_{false};
};

struct CanopenPdoMappingEntry {
  std::uint16_t index{0};
  std::uint8_t subindex{0};
  std::uint8_t bit_length{0};
};

template <std::size_t MaxEntries>
struct CanopenPdoMapping {
  static_assert(MaxEntries > 0U, "PDO mapping capacity must be positive");
  std::array<CanopenPdoMappingEntry, MaxEntries> entries{};
  std::size_t count{0U};

  [[nodiscard]] constexpr bool valid() const noexcept {
    if (count == 0U || count > MaxEntries) return false;
    std::size_t total_bits = 0U;
    for (std::size_t i = 0; i < count; ++i) {
      const auto& entry = entries[i];
      if (entry.index == 0U || entry.bit_length == 0U ||
          (entry.bit_length != 8U && entry.bit_length != 16U &&
           entry.bit_length != 32U)) {
        return false;
      }
      total_bits += entry.bit_length;
    }
    return total_bits <= 64U;
  }
};

template <std::size_t MaxObjects>
class CanopenVirtualNode {
 public:
  static_assert(MaxObjects > 0U, "Object dictionary capacity must be positive");

  struct Object {
    std::uint16_t index{0};
    std::uint8_t subindex{0};
    std::uint32_t value{0};
    std::uint8_t size_bytes{4};
    bool readable{true};
    bool writable{true};
  };

  constexpr explicit CanopenVirtualNode(std::uint8_t node_id = 1U) noexcept
      : node_id_(node_id) {}

  [[nodiscard]] constexpr bool valid() const noexcept {
    return node_id_ > 0U && node_id_ <= 127U;
  }

  [[nodiscard]] constexpr std::uint8_t node_id() const noexcept {
    return node_id_;
  }
  void complete_initialization() noexcept {
    if (nmt_state_ == CanopenNmtState::Initialising)
      nmt_state_ = CanopenNmtState::PreOperational;
  }
  [[nodiscard]] constexpr CanopenNmtState nmt_state() const noexcept {
    return nmt_state_;
  }
  [[nodiscard]] constexpr bool communication_faulted() const noexcept {
    return communication_faulted_;
  }

  core::Status add_object(const Object& object) noexcept {
    if (!valid() || object.index == 0U ||
        (object.size_bytes != 1U && object.size_bytes != 2U &&
         object.size_bytes != 4U) || object_count_ >= MaxObjects ||
        find_object(object.index, object.subindex) != nullptr) {
      return {core::StatusCode::InvalidArgument};
    }
    objects_[object_count_++] = object;
    return {};
  }

  void set_rpdo1_mapping(const CanopenPdoMapping<4U>& mapping) noexcept {
    rpdo1_mapping_ = mapping;
  }
  void set_tpdo1_mapping(const CanopenPdoMapping<4U>& mapping) noexcept {
    tpdo1_mapping_ = mapping;
  }

  // Process one received frame and return at most one response. NMT, SDO and
  // RPDO handling are deterministic and allocation-free for SIL tests.
  core::Status process(const CanopenCanFrame& request,
                       CanopenCanFrame& response,
                       bool& has_response) noexcept {
    has_response = false;
    if (!valid() || !request.valid()) {
      return {core::StatusCode::InvalidArgument};
    }
    if (request.cob_id == 0x000U) {
      return process_nmt(request);
    }
    if (request.cob_id == static_cast<std::uint16_t>(0x600U + node_id_)) {
      if (nmt_state_ == CanopenNmtState::Stopped ||
          nmt_state_ == CanopenNmtState::Initialising) {
        return {core::StatusCode::RuntimeError};
      }
      response = process_sdo(request);
      has_response = true;
      return {};
    }
    if (request.cob_id == static_cast<std::uint16_t>(0x200U + node_id_)) {
      if (nmt_state_ != CanopenNmtState::Operational ||
          !rpdo1_mapping_.valid() || request.dlc != mapped_bytes(rpdo1_mapping_)) {
        return {core::StatusCode::RuntimeError};
      }
      return apply_pdo(rpdo1_mapping_, request);
    }
    return {core::StatusCode::ConstraintViolation};
  }

  [[nodiscard]] CanopenCanFrame heartbeat() const noexcept {
    CanopenCanFrame frame;
    frame.cob_id = static_cast<std::uint16_t>(0x700U + node_id_);
    frame.dlc = 1U;
    frame.data[0] = static_cast<std::uint8_t>(nmt_state_);
    return frame;
  }

  [[nodiscard]] bool make_tpdo1(CanopenCanFrame& frame) const noexcept {
    if (nmt_state_ != CanopenNmtState::Operational ||
        !tpdo1_mapping_.valid()) return false;
    CanopenCanFrame candidate;
    candidate.cob_id = static_cast<std::uint16_t>(0x180U + node_id_);
    candidate.dlc = static_cast<std::uint8_t>(mapped_bytes(tpdo1_mapping_));
    std::size_t offset = 0U;
    for (std::size_t i = 0; i < tpdo1_mapping_.count; ++i) {
      const auto& map = tpdo1_mapping_.entries[i];
      const auto* object = find_object(map.index, map.subindex);
      if (object == nullptr || !object->readable ||
          object->size_bytes * 8U != map.bit_length) return false;
      for (std::size_t b = 0; b < object->size_bytes; ++b) {
        candidate.data[offset++] =
            static_cast<std::uint8_t>((object->value >> (8U * b)) & 0xFFU);
      }
    }
    frame = candidate;
    return true;
  }

  void set_bus_off() noexcept {
    communication_faulted_ = true;
  }
  void set_bus_recovered() noexcept {
    communication_faulted_ = false;
  }

 private:
  static constexpr std::uint8_t kSdoAbort = 0x80U;

  [[nodiscard]] static constexpr std::uint8_t mapped_bytes(
      const CanopenPdoMapping<4U>& mapping) noexcept {
    std::size_t bits = 0U;
    for (std::size_t i = 0; i < mapping.count && i < 4U; ++i)
      bits += mapping.entries[i].bit_length;
    return static_cast<std::uint8_t>(bits / 8U);
  }

  Object* find_object(std::uint16_t index, std::uint8_t subindex) noexcept {
    for (std::size_t i = 0; i < object_count_; ++i) {
      if (objects_[i].index == index && objects_[i].subindex == subindex)
        return &objects_[i];
    }
    return nullptr;
  }
  const Object* find_object(std::uint16_t index,
                            std::uint8_t subindex) const noexcept {
    for (std::size_t i = 0; i < object_count_; ++i) {
      if (objects_[i].index == index && objects_[i].subindex == subindex)
        return &objects_[i];
    }
    return nullptr;
  }

  core::Status process_nmt(const CanopenCanFrame& frame) noexcept {
    if (frame.dlc != 2U || frame.data[1] > 127U ||
        (frame.data[1] != 0U && frame.data[1] != node_id_)) {
      return {core::StatusCode::InvalidArgument};
    }
    const auto command = frame.data[0];
    if (nmt_state_ == CanopenNmtState::Initialising &&
        command != 0x81U && command != 0x82U) {
      return {core::StatusCode::RuntimeError};
    }
    switch (command) {
      case 0x01U: nmt_state_ = CanopenNmtState::Operational; break;
      case 0x02U: nmt_state_ = CanopenNmtState::Stopped; break;
      case 0x80U: nmt_state_ = CanopenNmtState::PreOperational; break;
      case 0x81U:
      case 0x82U:
        nmt_state_ = CanopenNmtState::Initialising;
        communication_faulted_ = false;
        break;
      default: return {core::StatusCode::InvalidArgument};
    }
    return {};
  }

  CanopenCanFrame abort_sdo(const CanopenCanFrame& request,
                            std::uint32_t code) const noexcept {
    CanopenCanFrame response;
    response.cob_id = static_cast<std::uint16_t>(0x580U + node_id_);
    response.dlc = 8U;
    response.data[0] = kSdoAbort;
    response.data[1] = request.data[1];
    response.data[2] = request.data[2];
    response.data[3] = request.data[3];
    for (std::size_t i = 0; i < 4U; ++i)
      response.data[4U + i] = static_cast<std::uint8_t>((code >> (8U * i)) & 0xFFU);
    return response;
  }

  CanopenCanFrame process_sdo(const CanopenCanFrame& request) noexcept {
    if (request.dlc != 8U) return abort_sdo(request, 0x05040001U);
    const auto command = request.data[0];
    const auto index = static_cast<std::uint16_t>(request.data[1]) |
                       (static_cast<std::uint16_t>(request.data[2]) << 8U);
    const auto subindex = request.data[3];
    auto* object = find_object(index, subindex);
    if (object == nullptr) return abort_sdo(request, 0x06020000U);

    CanopenCanFrame response;
    response.cob_id = static_cast<std::uint16_t>(0x580U + node_id_);
    response.dlc = 8U;
    response.data[1] = request.data[1];
    response.data[2] = request.data[2];
    response.data[3] = subindex;

    if (command == 0x40U) {
      if (!object->readable) return abort_sdo(request, 0x06010001U);
      response.data[0] = object->size_bytes == 4U ? 0x43U :
                         object->size_bytes == 2U ? 0x4BU : 0x4FU;
      for (std::size_t i = 0; i < object->size_bytes; ++i)
        response.data[4U + i] =
            static_cast<std::uint8_t>((object->value >> (8U * i)) & 0xFFU);
      return response;
    }

    std::uint8_t size = 0U;
    if (command == 0x2FU) size = 1U;
    else if (command == 0x2BU) size = 2U;
    else if (command == 0x23U) size = 4U;
    else return abort_sdo(request, 0x05040001U);
    if (!object->writable) return abort_sdo(request, 0x06010002U);
    if (size != object->size_bytes) return abort_sdo(request, 0x06070010U);
    std::uint32_t value = 0U;
    for (std::size_t i = 0; i < size; ++i)
      value |= static_cast<std::uint32_t>(request.data[4U + i]) << (8U * i);
    object->value = value;
    response.data[0] = 0x60U;
    return response;
  }

  core::Status apply_pdo(const CanopenPdoMapping<4U>& mapping,
                         const CanopenCanFrame& frame) noexcept {
    std::array<Object*, 4U> targets{};
    std::size_t offset = 0U;
    // Validate every mapped object before mutating any object: RPDO is atomic
    // at this virtual-node boundary even when a malformed map is supplied.
    for (std::size_t i = 0; i < mapping.count; ++i) {
      const auto& map = mapping.entries[i];
      targets[i] = find_object(map.index, map.subindex);
      if (targets[i] == nullptr || !targets[i]->writable ||
          targets[i]->size_bytes * 8U != map.bit_length) {
        return {core::StatusCode::ConstraintViolation};
      }
      offset += targets[i]->size_bytes;
    }
    if (offset != frame.dlc) return {core::StatusCode::InvalidArgument};
    offset = 0U;
    for (std::size_t i = 0; i < mapping.count; ++i) {
      std::uint32_t value = 0U;
      for (std::size_t b = 0; b < targets[i]->size_bytes; ++b)
        value |= static_cast<std::uint32_t>(frame.data[offset++]) << (8U * b);
      targets[i]->value = value;
    }
    return {};
  }

  std::uint8_t node_id_{1U};
  CanopenNmtState nmt_state_{CanopenNmtState::PreOperational};
  std::array<Object, MaxObjects> objects_{};
  std::size_t object_count_{0U};
  CanopenPdoMapping<4U> rpdo1_mapping_{};
  CanopenPdoMapping<4U> tpdo1_mapping_{};
  bool communication_faulted_{false};
};

}  // namespace mca::hardware
