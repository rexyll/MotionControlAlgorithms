#include "mca_ros2_control_adapter/system_hardware.hpp"

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
                     [&name](const auto& item) { return item.name == name; });
}

}  // namespace

SystemHardware::CallbackReturn SystemHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  const auto parent_result = hardware_interface::SystemInterface::on_init(params);
  if (parent_result != CallbackReturn::SUCCESS) return parent_result;

  const auto& info = params.hardware_info;
  if (info.type != "system" || info.joints.size() != 2U) {
    return CallbackReturn::ERROR;
  }

  has_effort_state_ = false;
  bool effort_state_topology_initialized = false;
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const auto& joint = info.joints[i];
    if (joint.name.empty() || joint.command_interfaces.size() != 1U ||
        !HasInterface(joint.command_interfaces, hardware_interface::HW_IF_EFFORT) ||
        !HasInterface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
        !HasInterface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY)) {
      return CallbackReturn::ERROR;
    }

    bool joint_has_effort_state = false;
    for (const auto& state : joint.state_interfaces) {
      if (state.name == hardware_interface::HW_IF_POSITION ||
          state.name == hardware_interface::HW_IF_VELOCITY) {
        continue;
      }
      if (state.name == hardware_interface::HW_IF_EFFORT) {
        joint_has_effort_state = true;
        continue;
      }
      return CallbackReturn::ERROR;
    }
    if (effort_state_topology_initialized &&
        joint_has_effort_state != has_effort_state_) {
      return CallbackReturn::ERROR;
    }
    has_effort_state_ = joint_has_effort_state;
    effort_state_topology_initialized = true;

    joint_names_[i] = joint.name;
    position_names_[i] = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    velocity_names_[i] = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
    effort_state_names_[i] = joint.name + "/" + hardware_interface::HW_IF_EFFORT;
    effort_command_names_[i] = joint.name + "/" + hardware_interface::HW_IF_EFFORT;
  }

  if (joint_names_[0] == joint_names_[1]) return CallbackReturn::ERROR;

  backend_ = create_backend();
  if (!backend_) return CallbackReturn::ERROR;
  active_.store(false, std::memory_order_release);
  has_last_state_sequence_ = false;
  last_state_sequence_ = 0;
  next_command_sequence_ = 0;
  position_handles_ = {};
  velocity_handles_ = {};
  effort_state_handles_ = {};
  effort_command_handles_ = {};
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
SystemHardware::on_export_state_interfaces() {
  auto handles = hardware_interface::SystemInterface::on_export_state_interfaces();
  try {
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      position_handles_[i] = get_state_interface_handle(position_names_[i]);
      velocity_handles_[i] = get_state_interface_handle(velocity_names_[i]);
      if (has_effort_state_) {
        effort_state_handles_[i] = get_state_interface_handle(effort_state_names_[i]);
      }
    }
  } catch (...) {
    position_handles_ = {};
    velocity_handles_ = {};
    effort_state_handles_ = {};
  }
  return handles;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
SystemHardware::on_export_command_interfaces() {
  auto handles = hardware_interface::SystemInterface::on_export_command_interfaces();
  try {
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      effort_command_handles_[i] = get_command_interface_handle(effort_command_names_[i]);
    }
  } catch (...) {
    effort_command_handles_ = {};
  }
  return handles;
}

SystemHardware::CallbackReturn SystemHardware::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!backend_) backend_ = create_backend();
  if (!backend_ || !backend_->initialize().ok()) return CallbackReturn::FAILURE;
  has_last_state_sequence_ = false;
  last_state_sequence_ = 0;
  next_command_sequence_ = 0;
  return CallbackReturn::SUCCESS;
}

SystemHardware::CallbackReturn SystemHardware::on_activate(
    const rclcpp_lifecycle::State&) {
  if (!backend_ || !backend_->activate().ok()) return CallbackReturn::ERROR;
  active_.store(true, std::memory_order_release);
  return CallbackReturn::SUCCESS;
}

SystemHardware::CallbackReturn SystemHardware::stop_and_deactivate() noexcept {
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

SystemHardware::CallbackReturn SystemHardware::on_deactivate(
    const rclcpp_lifecycle::State&) {
  return stop_and_deactivate();
}

SystemHardware::CallbackReturn SystemHardware::on_cleanup(
    const rclcpp_lifecycle::State&) {
  if (active_.load(std::memory_order_acquire)) return CallbackReturn::ERROR;
  backend_.reset();
  return CallbackReturn::SUCCESS;
}

SystemHardware::CallbackReturn SystemHardware::on_shutdown(
    const rclcpp_lifecycle::State&) {
  return stop_and_deactivate();
}

SystemHardware::CallbackReturn SystemHardware::on_error(
    const rclcpp_lifecycle::State&) {
  return stop_and_deactivate();
}

bool SystemHardware::validate_backend_state(const DualJointState& state) const noexcept {
  return state.valid() &&
         state.has(mca::hardware::JointStateField::Position) &&
         state.has(mca::hardware::JointStateField::Velocity) &&
         (!has_effort_state_ ||
          state.has(mca::hardware::JointStateField::Effort));
}

hardware_interface::return_type SystemHardware::read(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  if (!backend_) return hardware_interface::return_type::ERROR;
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    if (!position_handles_[i] || !velocity_handles_[i] ||
        (has_effort_state_ && !effort_state_handles_[i])) {
      return hardware_interface::return_type::ERROR;
    }
  }

  const double period_seconds = period.seconds();
  if (!std::isfinite(period_seconds) || period_seconds < 0.0 ||
      !backend_->update(period_seconds).ok()) {
    return hardware_interface::return_type::ERROR;
  }

  DualJointState sample;
  if (!backend_->read_state(sample).ok() || !validate_backend_state(sample) ||
      (has_last_state_sequence_ && sample.sequence <= last_state_sequence_)) {
    return hardware_interface::return_type::ERROR;
  }

  // Validate the complete backend sample before publishing any joint value.
  try {
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      if (!set_state(position_handles_[i], sample.position[i], false) ||
          !set_state(velocity_handles_[i], sample.velocity[i], false) ||
          (has_effort_state_ &&
           !set_state(effort_state_handles_[i], sample.effort[i], false))) {
        return hardware_interface::return_type::ERROR;
      }
    }
  } catch (...) {
    return hardware_interface::return_type::ERROR;
  }

  last_state_sequence_ = sample.sequence;
  has_last_state_sequence_ = true;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SystemHardware::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
  if (!backend_ || !active_.load(std::memory_order_acquire) ||
      next_command_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    return hardware_interface::return_type::ERROR;
  }

  DualJointEffortCommand command;
  command.sequence = next_command_sequence_;
  try {
    // Read and validate the complete command before making a single backend call.
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      if (!effort_command_handles_[i] ||
          !get_command(effort_command_handles_[i], command.effort[i], false)) {
        return hardware_interface::return_type::ERROR;
      }
    }
  } catch (...) {
    return hardware_interface::return_type::ERROR;
  }

  if (!command.valid() || !backend_->write_effort(command).ok()) {
    return hardware_interface::return_type::ERROR;
  }
  ++next_command_sequence_;
  return hardware_interface::return_type::OK;
}

}  // namespace mca_ros2_control_adapter
