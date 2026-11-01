#include "mca_ros2_control_adapter/actuator_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace mca_ros2_control_adapter {
namespace {

bool HasInterface(const std::vector<hardware_interface::InterfaceInfo>& interfaces,
                  const std::string& name) {
  return std::any_of(interfaces.begin(), interfaces.end(),
                     [&name](const auto& interface_info) {
                       return interface_info.name == name;
                     });
}

}  // namespace

ActuatorHardware::CallbackReturn ActuatorHardware::on_init(
    const hardware_interface::HardwareInfo& info) {
  const auto parent_result = hardware_interface::ActuatorInterface::on_init(info);
  if (parent_result != CallbackReturn::SUCCESS) return parent_result;

  if (info.type != "actuator" || info.joints.size() != 1U ||
      info.joints.front().name.empty()) {
    return CallbackReturn::ERROR;
  }

  const auto& joint = info.joints.front();
  if (joint.command_interfaces.size() != 1U ||
      !HasInterface(joint.command_interfaces, hardware_interface::HW_IF_EFFORT)) {
    return CallbackReturn::ERROR;
  }

  if (!HasInterface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !HasInterface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY)) {
    return CallbackReturn::ERROR;
  }

  for (const auto& state_interface : joint.state_interfaces) {
    if (state_interface.name == hardware_interface::HW_IF_POSITION ||
        state_interface.name == hardware_interface::HW_IF_VELOCITY) {
      continue;
    }
    if (state_interface.name == hardware_interface::HW_IF_ACCELERATION) {
      has_acceleration_state_ = true;
      continue;
    }
    if (state_interface.name == hardware_interface::HW_IF_EFFORT) {
      has_effort_state_ = true;
      continue;
    }
    return CallbackReturn::ERROR;
  }

  joint_name_ = joint.name;
  backend_ = create_backend();
  if (!backend_) return CallbackReturn::ERROR;
  next_command_sequence_ = 0;
  return CallbackReturn::SUCCESS;
}

ActuatorHardware::CallbackReturn ActuatorHardware::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!backend_) backend_ = create_backend();
  if (!backend_ || !backend_->initialize().ok()) return CallbackReturn::FAILURE;
  return CallbackReturn::SUCCESS;
}

ActuatorHardware::CallbackReturn ActuatorHardware::on_activate(
    const rclcpp_lifecycle::State&) {
  if (!backend_ || !backend_->activate().ok()) return CallbackReturn::ERROR;
  return CallbackReturn::SUCCESS;
}

ActuatorHardware::CallbackReturn ActuatorHardware::stop_and_deactivate() noexcept {
  if (!backend_) return CallbackReturn::SUCCESS;
  if (backend_->lifecycle_state() == mca::hardware::HardwareLifecycleState::Active) {
    if (!backend_->request_safe_stop().ok()) return CallbackReturn::ERROR;
    if (!backend_->deactivate().ok()) return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

ActuatorHardware::CallbackReturn ActuatorHardware::on_deactivate(
    const rclcpp_lifecycle::State&) {
  return stop_and_deactivate();
}

ActuatorHardware::CallbackReturn ActuatorHardware::on_cleanup(
    const rclcpp_lifecycle::State&) {
  if (backend_ &&
      backend_->lifecycle_state() == mca::hardware::HardwareLifecycleState::Active) {
    return CallbackReturn::ERROR;
  }
  backend_.reset();
  return CallbackReturn::SUCCESS;
}

ActuatorHardware::CallbackReturn ActuatorHardware::on_shutdown(
    const rclcpp_lifecycle::State&) {
  return stop_and_deactivate();
}

ActuatorHardware::CallbackReturn ActuatorHardware::on_error(
    const rclcpp_lifecycle::State&) {
  return stop_and_deactivate();
}

bool ActuatorHardware::validate_backend_state(
    const mca::hardware::ScalarJointState& state) const noexcept {
  return state.valid() && state.has(mca::hardware::JointStateField::Position) &&
         state.has(mca::hardware::JointStateField::Velocity) &&
         (!has_acceleration_state_ ||
          state.has(mca::hardware::JointStateField::Acceleration)) &&
         (!has_effort_state_ || state.has(mca::hardware::JointStateField::Effort));
}

hardware_interface::return_type ActuatorHardware::read(
    const rclcpp::Time&, const rclcpp::Duration&) {
  if (!backend_) return hardware_interface::return_type::ERROR;

  mca::hardware::ScalarJointState state;
  if (!backend_->read_state(state).ok() || !validate_backend_state(state)) {
    return hardware_interface::return_type::ERROR;
  }

  set_state(joint_name_ + "/" + hardware_interface::HW_IF_POSITION, state.position[0]);
  set_state(joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY, state.velocity[0]);
  if (has_acceleration_state_) {
    set_state(joint_name_ + "/" + hardware_interface::HW_IF_ACCELERATION,
              state.acceleration[0]);
  }
  if (has_effort_state_) {
    set_state(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT, state.effort[0]);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ActuatorHardware::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
  if (!backend_ ||
      backend_->lifecycle_state() != mca::hardware::HardwareLifecycleState::Active ||
      next_command_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    return hardware_interface::return_type::ERROR;
  }

  const double effort =
      get_command<double>(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT);
  mca::hardware::ScalarJointEffortCommand command;
  command.effort[0] = effort;
  command.sequence = next_command_sequence_;
  if (!command.valid() || !backend_->write_effort(command).ok()) {
    return hardware_interface::return_type::ERROR;
  }
  ++next_command_sequence_;
  return hardware_interface::return_type::OK;
}

}  // namespace mca_ros2_control_adapter
