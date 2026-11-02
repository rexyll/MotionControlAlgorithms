#pragma once

// Optional Linux-only SocketCAN transport. Enable with
// -DMCA_ENABLE_SOCKETCAN=ON; this header is intentionally not included by
// mca/mca.hpp so the portable core never acquires Linux kernel dependencies.
#if !defined(MCA_ENABLE_SOCKETCAN) || !defined(__linux__)
#error "SocketCAN transport requires Linux and MCA_ENABLE_SOCKETCAN=ON"
#endif

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "mca/hardware/canopen.hpp"

namespace mca::hardware {

class SocketCanTransport final : public ICanopenTransport {
 public:
  explicit SocketCanTransport(std::string_view interface_name) noexcept {
    if (interface_name.empty() || interface_name.size() >= IFNAMSIZ ||
        interface_name.find('\0') != std::string_view::npos) {
      status_ = {core::StatusCode::InvalidArgument};
      return;
    }

    std::array<char, IFNAMSIZ> name{};
    std::memcpy(name.data(), interface_name.data(), interface_name.size());

    socket_fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC,
                          CAN_RAW);
    if (socket_fd_ < 0) {
      status_ = {core::StatusCode::RuntimeError};
      return;
    }

    can_err_mask_t error_mask = CAN_ERR_BUSOFF | CAN_ERR_RESTARTED;
    if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &error_mask,
                     sizeof(error_mask)) < 0) {
      close_socket();
      status_ = {core::StatusCode::RuntimeError};
      return;
    }

    struct ifreq request {};
    std::memcpy(request.ifr_name, name.data(), name.size());
    if (::ioctl(socket_fd_, SIOCGIFINDEX, &request) < 0) {
      close_socket();
      status_ = {core::StatusCode::RuntimeError};
      return;
    }

    struct sockaddr_can address {};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&address),
               sizeof(address)) < 0) {
      close_socket();
      status_ = {core::StatusCode::RuntimeError};
      return;
    }

    state_ = CanopenTransportState::Online;
    status_ = {};
  }

  ~SocketCanTransport() override { close_socket(); }

  SocketCanTransport(const SocketCanTransport&) = delete;
  SocketCanTransport& operator=(const SocketCanTransport&) = delete;
  SocketCanTransport(SocketCanTransport&&) = delete;
  SocketCanTransport& operator=(SocketCanTransport&&) = delete;

  [[nodiscard]] core::Status initialization_status() const noexcept {
    return status_;
  }

  [[nodiscard]] bool is_open() const noexcept { return socket_fd_ >= 0; }

  core::Status send(const CanopenCanFrame& frame) noexcept override {
    if (!frame.valid()) return {core::StatusCode::InvalidArgument};
    if (socket_fd_ < 0 || state_ != CanopenTransportState::Online)
      return {core::StatusCode::RuntimeError};

    struct can_frame native_frame {};
    native_frame.can_id = frame.cob_id;
    native_frame.len = frame.dlc;
    for (std::size_t i = 0; i < frame.data.size(); ++i)
      native_frame.data[i] = frame.data[i];

    ssize_t written = -1;
    do {
      written = ::write(socket_fd_, &native_frame, sizeof(native_frame));
    } while (written < 0 && errno == EINTR);

    if (written != static_cast<ssize_t>(sizeof(native_frame))) {
      if (written < 0 && (errno == ENETDOWN || errno == ENODEV))
        state_ = CanopenTransportState::Offline;
      return {core::StatusCode::RuntimeError};
    }
    return {};
  }

  core::Status receive(CanopenCanFrame& frame,
                       bool& has_frame) noexcept override {
    has_frame = false;
    if (socket_fd_ < 0 || state_ == CanopenTransportState::Offline)
      return {core::StatusCode::RuntimeError};

    struct can_frame native_frame {};
    ssize_t received = -1;
    do {
      received = ::read(socket_fd_, &native_frame, sizeof(native_frame));
    } while (received < 0 && errno == EINTR);

    if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return {};
    if (received != static_cast<ssize_t>(sizeof(native_frame))) {
      if (received < 0 && (errno == ENETDOWN || errno == ENODEV))
        state_ = CanopenTransportState::Offline;
      return {core::StatusCode::RuntimeError};
    }

    if ((native_frame.can_id & CAN_ERR_FLAG) != 0U) {
      if ((native_frame.can_id & CAN_ERR_BUSOFF) != 0U)
        state_ = CanopenTransportState::BusOff;
      if ((native_frame.can_id & CAN_ERR_RESTARTED) != 0U)
        state_ = CanopenTransportState::Online;
      return {};
    }

    if ((native_frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) != 0U ||
        native_frame.len > CAN_MAX_DLEN ||
        (native_frame.can_id & CAN_SFF_MASK) > 0x07FFU) {
      return {core::StatusCode::ConstraintViolation};
    }

    CanopenCanFrame candidate;
    candidate.cob_id =
        static_cast<std::uint16_t>(native_frame.can_id & CAN_SFF_MASK);
    candidate.dlc = native_frame.len;
    for (std::size_t i = 0; i < candidate.data.size(); ++i)
      candidate.data[i] = native_frame.data[i];
    if (!candidate.valid()) return {core::StatusCode::ConstraintViolation};

    frame = candidate;
    has_frame = true;
    return {};
  }

  [[nodiscard]] CanopenTransportState state() const noexcept override {
    return state_;
  }

 private:
  void close_socket() noexcept {
    if (socket_fd_ >= 0) {
      (void)::close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  int socket_fd_{-1};
  CanopenTransportState state_{CanopenTransportState::Offline};
  core::Status status_{core::StatusCode::NotConfigured};
};

}  // namespace mca::hardware
