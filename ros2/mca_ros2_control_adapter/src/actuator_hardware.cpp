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
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  const auto parent_result = hardware_interface::ActuatorInterface::on_init(params);
  if (parent_result != CallbackReturn::SUCCESS) return parent_result;

  const auto& info = params.hardware_info;
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

  has_acceleration_state_ = false;
  has_effort_state_ = false;
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
  position_state_interface_name_ = joint_name_ + "/" + hardware_interface::HW_IF_POSITION;
  velocity_state_interface_name_ = joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY;
  acceleration_state_interface_name_ =
      joint_name_ + "/" + hardware_interface::HW_IF_ACCELERATION;
  effort_state_interface_name_ = joint_name_ + "/" + hardware_interface::HW_IF_EFFORT;
  effort_command_interface_name_ = joint_name_ + "/" + hardware_interface::HW_IF_EFFORT;

  backend_ = create_backend();
  if (!backend_) return CallbackReturn::ERROR;
  active_.store(false, std::memory_order_release);
  has_last_state_sequence_ = false;
  last_state_sequence_ = 0;
  next_command_sequence_ = 0;
  position_state_handle_.reset();
  velocity_state_handle_.reset();
  acceleration_state_handle_.reset();
  effort_state_handle_.reset();
  effort_command_handle_.reset();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
ActuatorHardware::on_export_state_interfaces() {
  auto handles = hardware_interface::ActuatorInterface::on_export_state_interfaces();
  try {
    position_state_handle_ = get_state_interface_handle(position_state_interface_name_);
    velocity_state_handle_ = get_state_interface_handle(velocity_state_interface_name_);
    if (has_acceleration_state_) {
      acceleration_state_handle_ =
          get_state_interface_handle(acceleration_state_interface_name_);
    }
    if (has_effort_state_) {
      effort_state_handle_ = get_state_interface_handle(effort_state_interface_name_);
    }
  } catch (...) {
    position_state_handle_.reset();
    velocity_state_handle_.reset();
    acceleration_state_handle_.reset();
    effort_state_handle_.reset();
  }
  return handles;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
ActuatorHardware::on_export_command_interfaces() {
  auto handles = hardware_interface::ActuatorInterface::on_export_command_interfaces();
  try {
    effort_command_handle_ = get_command_interface_handle(effort_command_interface_name_);
  } catch (...) {
    effort_command_handle_.reset();
  }
  return handles;
}

ActuatorHardware::CallbackReturn ActuatorHardware::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!backend_) backend_ = create_backend();
  if (!backend_ || !backend_->initialize().ok()) return CallbackReturn::FAILURE;
  has_last_state_sequence_ = false;
  last_state_sequence_ = 0;
  next_command_sequence_ = 0;
  return CallbackReturn::SUCCESS;
}

ActuatorHardware::CallbackReturn ActuatorHardware::on_activate(
    const rclcpp_lifecycle::State&) {
  if (!backend_ || !backend_->activate().ok()) return CallbackReturn::ERROR;
  active_.store(true, std::memory_order_release);
  return CallbackReturn::SUCCESS;
}

ActuatorHardware::CallbackReturn ActuatorHardware::stop_and_deactivate() noexcept {
  if (!backend_) {
    active_.store(false, std::memory_order_release);
    return CallbackReturn::SUCCESS;
  }
  if (active_.load(std::memory_order_acquire)) {
    if (!backend_->request_safe_stop().ok()) return CallbackReturn::ERROR;
    if (!backend_->deactivate().ok()) return CallbackReturn::ERROR;
    active_.store(false, std::memory_order_release);
  }
  return CallbackReturn::SUCCESS;
}

ActuatorHardware::CallbackReturn ActuatorHardware::on_deactivate(
    const rclcpp_lifecycle::State&) {
  return stop_and_deactivate();
}

ActuatorHardware::CallbackReturn ActuatorHardware::on_cleanup(
    const rclcpp_lifecycle::State&) {
  if (active_.load(std::memory_order_acquire)) return CallbackReturn::ERROR;
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
  if (!backend_ || !position_state_handle_ || !velocity_state_handle_ ||
      (has_acceleration_state_ && !acceleration_state_handle_) ||
      (has_effort_state_ && !effort_state_handle_)) {
    return hardware_interface::return_type::ERROR;
  }

  mca::hardware::ScalarJointState state;
  if (!backend_->read_state(state).ok() || !validate_backend_state(state)) {
    return hardware_interface::return_type::ERROR;
  }
  if (has_last_state_sequence_ && state.sequence <= last_state_sequence_) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    // All handles and names were resolved during interface export. Use the
    // non-blocking handle overloads in the control cycle; never construct
    // strings, perform name lookups, or wait for interface storage here.
    if (!set_state(position_state_handle_, state.position[0], false) ||
        !set_state(velocity_state_handle_, state.velocity[0], false)) {
      return hardware_interface::return_type::ERROR;
    }
    if (has_acceleration_state_ &&
        !set_state(acceleration_state_handle_, state.acceleration[0], false)) {
      return hardware_interface::return_type::ERROR;
    }
    if (has_effort_state_ &&
        !set_state(effort_state_handle_, state.effort[0], false)) {
      return hardware_interface::return_type::ERROR;
    }
  } catch (...) {
    return hardware_interface::return_type::ERROR;
  }

  last_state_sequence_ = state.sequence;
  has_last_state_sequence_ = true;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ActuatorHardware::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
  if (!backend_ || !active_.load(std::memory_order_acquire) ||
      !effort_command_handle_ ||
      next_command_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    return hardware_interface::return_type::ERROR;
  }

  double effort = 0.0;
  try {
    if (!get_command(effort_command_handle_, effort, false)) {
      return hardware_interface::return_type::ERROR;
    }
  } catch (...) {
    return hardware_interface::return_type::ERROR;
  }

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
