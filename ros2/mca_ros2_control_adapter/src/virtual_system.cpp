#include "mca_ros2_control_adapter/virtual_system.hpp"

#include <cmath>
#include <memory>

#include "mca/hardware/virtual_joint_device.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mca_ros2_control_adapter {
namespace {

class VirtualDualJointBackend final : public IDualJointBackend {
 public:
  using Device = mca::hardware::VirtualJointDevice<2>;
  using Lifecycle = mca::hardware::HardwareLifecycleState;

  mca::core::Status initialize() noexcept override {
    return device_.initialize();
  }

  mca::core::Status activate() noexcept override {
    return device_.activate();
  }

  mca::core::Status deactivate() noexcept override {
    return device_.deactivate();
  }

  Lifecycle lifecycle_state() const noexcept override {
    return device_.lifecycle_state();
  }

  mca::core::Status update(double period_seconds) noexcept override {
    if (!std::isfinite(period_seconds) || period_seconds < 0.0) {
      return {mca::core::StatusCode::InvalidArgument};
    }
    if (period_seconds == 0.0 || lifecycle_state() != Lifecycle::Active) {
      return {};
    }
    // The virtual plant intentionally rejects periods larger than its
    // configured integration bound instead of silently substepping.
    return device_.step(period_seconds);
  }

  mca::core::Status read_state(DualJointState& output) noexcept override {
    return device_.read_state(output);
  }

  mca::core::Status write_effort(
      const DualJointEffortCommand& command) noexcept override {
    return device_.write_effort(command);
  }

  mca::core::Status request_safe_stop() noexcept override {
    return device_.request_safe_stop();
  }

 private:
  Device device_{};
};

}  // namespace

std::unique_ptr<IDualJointBackend> VirtualSystem::create_backend() {
  return std::make_unique<VirtualDualJointBackend>();
}

}  // namespace mca_ros2_control_adapter

PLUGINLIB_EXPORT_CLASS(mca_ros2_control_adapter::VirtualSystem,
                       hardware_interface::SystemInterface)
