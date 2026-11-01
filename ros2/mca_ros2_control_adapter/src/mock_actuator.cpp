#include "mca_ros2_control_adapter/mock_actuator.hpp"

#include <memory>

#include "pluginlib/class_list_macros.hpp"

namespace mca_ros2_control_adapter {
namespace {

class MockScalarBackend final : public IScalarJointBackend {
 public:
  using State = mca::hardware::ScalarJointState;
  using Command = mca::hardware::ScalarJointEffortCommand;
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
    applied_effort_ = 0.0;
    lifecycle_ = Lifecycle::Inactive;
    return {};
  }

  Lifecycle lifecycle_state() const noexcept override { return lifecycle_; }

  mca::core::Status read_state(State& output) noexcept override {
    if (lifecycle_ != Lifecycle::Inactive && lifecycle_ != Lifecycle::Active) {
      return {mca::core::StatusCode::NotConfigured};
    }
    State sample;
    sample.position[0] = 0.0;
    sample.velocity[0] = 0.0;
    sample.valid_fields = State::kPositionValid | State::kVelocityValid;
    sample.sequence = ++sample_sequence_;
    output = sample;
    return {};
  }

  mca::core::Status write_effort(const Command& command) noexcept override {
    if (lifecycle_ != Lifecycle::Active) {
      return {mca::core::StatusCode::NotConfigured};
    }
    if (!command.valid()) return {mca::core::StatusCode::InvalidArgument};
    applied_effort_ = command.effort[0];
    return {};
  }

  mca::core::Status request_safe_stop() noexcept override {
    if (lifecycle_ != Lifecycle::Active) {
      return {mca::core::StatusCode::NotConfigured};
    }
    applied_effort_ = 0.0;
    return {};
  }

 private:
  Lifecycle lifecycle_{Lifecycle::Uninitialized};
  std::uint64_t sample_sequence_{0};
  double applied_effort_{0.0};
};

}  // namespace

std::unique_ptr<IScalarJointBackend> MockActuator::create_backend() {
  return std::make_unique<MockScalarBackend>();
}

}  // namespace mca_ros2_control_adapter

PLUGINLIB_EXPORT_CLASS(mca_ros2_control_adapter::MockActuator,
                       hardware_interface::ActuatorInterface)
