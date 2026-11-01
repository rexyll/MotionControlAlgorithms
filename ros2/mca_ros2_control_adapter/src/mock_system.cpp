#include "mca_ros2_control_adapter/mock_system.hpp"

#include <memory>

#include "pluginlib/class_list_macros.hpp"

namespace mca_ros2_control_adapter {
namespace {

class MockDualJointBackend final : public IDualJointBackend {
 public:
  using Lifecycle = mca::hardware::HardwareLifecycleState;

  mca::core::Status initialize() noexcept override {
    if (lifecycle_ != Lifecycle::Uninitialized) {
      return {mca::core::StatusCode::InvalidArgument};
    }
    lifecycle_ = Lifecycle::Inactive;
    return {};
  }

  mca::core::Status activate() noexcept override {
    if (lifecycle_ != Lifecycle::Inactive) {
      return {mca::core::StatusCode::NotConfigured};
    }
    lifecycle_ = Lifecycle::Active;
    return {};
  }

  mca::core::Status deactivate() noexcept override {
    if (lifecycle_ != Lifecycle::Active) {
      return {mca::core::StatusCode::NotConfigured};
    }
    applied_effort_ = {};
    lifecycle_ = Lifecycle::Inactive;
    return {};
  }

  Lifecycle lifecycle_state() const noexcept override { return lifecycle_; }

  mca::core::Status read_state(DualJointState& output) noexcept override {
    if (lifecycle_ != Lifecycle::Inactive && lifecycle_ != Lifecycle::Active) {
      return {mca::core::StatusCode::NotConfigured};
    }
    DualJointState sample;
    sample.position = {0.25, -0.5};
    sample.velocity = {1.0, -1.0};
    sample.effort = applied_effort_;
    sample.valid_fields = DualJointState::kPositionValid |
                          DualJointState::kVelocityValid |
                          DualJointState::kEffortValid;
    sample.sequence = ++sample_sequence_;
    output = sample;
    return {};
  }

  mca::core::Status write_effort(const DualJointEffortCommand& command) noexcept override {
    if (lifecycle_ != Lifecycle::Active) {
      return {mca::core::StatusCode::NotConfigured};
    }
    if (!command.valid()) return {mca::core::StatusCode::InvalidArgument};
    applied_effort_ = command.effort;
    return {};
  }

  mca::core::Status request_safe_stop() noexcept override {
    if (lifecycle_ != Lifecycle::Active) {
      return {mca::core::StatusCode::NotConfigured};
    }
    applied_effort_ = {};
    return {};
  }

 private:
  Lifecycle lifecycle_{Lifecycle::Uninitialized};
  std::uint64_t sample_sequence_{0};
  std::array<double, 2> applied_effort_{};
};

}  // namespace

std::unique_ptr<IDualJointBackend> MockSystem::create_backend() {
  return std::make_unique<MockDualJointBackend>();
}

}  // namespace mca_ros2_control_adapter

PLUGINLIB_EXPORT_CLASS(mca_ros2_control_adapter::MockSystem,
                       hardware_interface::SystemInterface)
