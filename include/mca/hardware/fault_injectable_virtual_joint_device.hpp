#pragma once
#include <cstddef>
#include <cstdint>
#include "mca/hardware/diagnostics.hpp"
#include "mca/hardware/virtual_joint_device.hpp"

namespace mca::hardware {

// Deterministic simulation-only decorator. Injected faults inhibit commands,
// latch diagnostics, and require explicit resolve + clear + reactivation.
template <std::size_t N>
class FaultInjectableVirtualJointDevice final : public IHardwareLifecycle,
    public IJointStateSource<double, N>, public IJointEffortCommandSink<double, N>,
    public IHardwareDiagnostics {
 public:
  using Device = VirtualJointDevice<N>;
  using State = JointState<double, N>;
  using Command = JointEffortCommand<double, N>;
  explicit FaultInjectableVirtualJointDevice(const VirtualJointDeviceConfig<N>& c = {}) noexcept : device_(c) {}
  core::Status initialize() noexcept override {
    if (device_.lifecycle_state() != HardwareLifecycleState::Uninitialized) return {core::StatusCode::NotConfigured};
    auto s = device_.initialize(); if (!s.ok()) return s;
    faults_ = {}; next_sequence_ = 1; return {};
  }
  core::Status activate() noexcept override {
    if (faults_.count != 0) return {core::StatusCode::ConstraintViolation};
    return device_.activate();
  }
  core::Status deactivate() noexcept override {
    if (device_.lifecycle_state() != HardwareLifecycleState::Active) return {core::StatusCode::NotConfigured};
    auto s = device_.request_safe_stop(); if (!s.ok()) return s;
    return device_.deactivate();
  }
  HardwareLifecycleState lifecycle_state() const noexcept override {
    return faults_.count ? HardwareLifecycleState::Faulted : device_.lifecycle_state();
  }
  core::Status read_state(State& out) noexcept override { return device_.read_state(out); }
  core::Status write_effort(const Command& c) noexcept override {
    if (faults_.count) return {core::StatusCode::ConstraintViolation};
    return device_.write_effort(c);
  }
  core::Status request_safe_stop() noexcept override { return device_.request_safe_stop(); }
  core::Status step(double dt) noexcept {
    if (faults_.count) return {core::StatusCode::ConstraintViolation};
    return device_.step(dt);
  }
  core::Status read_faults(HardwareFaultSet<16>& out) noexcept override { out = faults_; return {}; }
  core::Status inject_fault(HardwareFaultCategory category, std::uint64_t raw_code,
                            std::uint64_t timestamp_ns) noexcept {
    if (device_.lifecycle_state() != HardwareLifecycleState::Active ||
        category > HardwareFaultCategory::Unknown || next_sequence_ == UINT64_MAX)
      return {core::StatusCode::InvalidArgument};
    for (std::size_t i=0; i<faults_.count; ++i)
      if (faults_.faults[i].category == category && faults_.faults[i].active)
        return {core::StatusCode::ConstraintViolation};
    if (faults_.count == faults_.faults.size()) return {core::StatusCode::ConstraintViolation};
    auto stop = device_.request_safe_stop(); if (!stop.ok()) return stop;
    auto& f = faults_.faults[faults_.count++];
    f.category=category; f.vendor_raw_code=raw_code; f.active=true; f.latched=true;
    f.sequence=next_sequence_++; f.timestamp_ns=timestamp_ns; return {};
  }
  core::Status resolve_fault(HardwareFaultCategory category) noexcept {
    for (std::size_t i=0; i<faults_.count; ++i) {
      auto& f=faults_.faults[i];
      if (f.category==category && f.active) { f.active=false; return {}; }
    }
    return {core::StatusCode::NotConfigured};
  }
  core::Status clear_faults() noexcept override {
    // Clearing an empty diagnostic set is a no-op. In particular, it must not
    // deactivate a healthy device merely because a caller polls diagnostics.
    if (faults_.count == 0U) return {};
    for (std::size_t i=0; i<faults_.count; ++i)
      if (faults_.faults[i].active) return {core::StatusCode::ConstraintViolation};
    if (device_.lifecycle_state()==HardwareLifecycleState::Active) {
      auto s=device_.deactivate(); if (!s.ok()) return s;
    }
    faults_={}; return {};
  }
 private:
  Device device_;
  HardwareFaultSet<16> faults_{};
  std::uint64_t next_sequence_{1};
};
}  // namespace mca::hardware
