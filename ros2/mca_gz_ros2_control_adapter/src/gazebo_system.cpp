#include "mca_gz_ros2_control_adapter/gazebo_system.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mca_gz_ros2_control_adapter {
namespace {

bool HasInterface(const std::vector<hardware_interface::InterfaceInfo>& interfaces,
                  const std::string& name) {
  return std::any_of(interfaces.begin(), interfaces.end(),
                     [&name](const auto& item) { return item.name == name; });
}

class GazeboEcmBackend final : public IDualJointBackend {
 public:
  using Lifecycle = mca::hardware::HardwareLifecycleState;

  bool bind(const std::array<gz::sim::Entity, 2>& joints,
            gz::sim::EntityComponentManager& ecm) noexcept override {
    if (bound_ || joints[0] == gz::sim::kNullEntity ||
        joints[1] == gz::sim::kNullEntity || joints[0] == joints[1]) {
      return false;
    }
    joints_ = joints;
    ecm_ = &ecm;
    for (const auto joint : joints_) {
      if (!ecm_->Component<gz::sim::components::JointPosition>(joint)) {
        ecm_->CreateComponent(joint, gz::sim::components::JointPosition());
      }
      if (!ecm_->Component<gz::sim::components::JointVelocity>(joint)) {
        ecm_->CreateComponent(joint, gz::sim::components::JointVelocity());
      }
      if (!ecm_->Component<gz::sim::components::JointForceCmd>(joint)) {
        ecm_->CreateComponent(joint, gz::sim::components::JointForceCmd());
      }
    }
    bound_ = true;
    return true;
  }

  mca::core::Status initialize() noexcept override {
    if (!bound_ || lifecycle_.load() != Lifecycle::Uninitialized) {
      return {mca::core::StatusCode::NotConfigured};
    }
    sequence_ = 0;
    last_command_sequence_ = 0;
    has_command_sequence_ = false;
    lifecycle_.store(Lifecycle::Inactive);
    return {};
  }

  mca::core::Status activate() noexcept override {
    if (lifecycle_.load() != Lifecycle::Inactive || safe_stop_requested_.load()) {
      return {mca::core::StatusCode::NotConfigured};
    }
    lifecycle_.store(Lifecycle::Active);
    return {};
  }

  mca::core::Status deactivate() noexcept override {
    if (lifecycle_.load() != Lifecycle::Active) {
      return {mca::core::StatusCode::NotConfigured};
    }
    safe_stop_requested_.store(true);
    lifecycle_.store(Lifecycle::Inactive);
    return {};
  }

  Lifecycle lifecycle_state() const noexcept override { return lifecycle_.load(); }

  mca::core::Status read_state(DualJointState& output) noexcept override {
    if (!bound_ || (lifecycle_.load() != Lifecycle::Inactive &&
                    lifecycle_.load() != Lifecycle::Active) || !ecm_) {
      return {mca::core::StatusCode::NotConfigured};
    }

    // Lifecycle callbacks run on the ROS executor, whereas ECM access is only
    // safe on Gazebo's simulation update thread. Apply deferred stops here.
    if (safe_stop_requested_.exchange(false) && !apply_zero_effort()) {
      safe_stop_requested_.store(true);
      return {mca::core::StatusCode::RuntimeError};
    }

    DualJointState sample;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      const auto* position =
          ecm_->Component<gz::sim::components::JointPosition>(joints_[i]);
      const auto* velocity =
          ecm_->Component<gz::sim::components::JointVelocity>(joints_[i]);
      if (!position || !velocity || position->Data().empty() ||
          velocity->Data().empty()) {
        return {mca::core::StatusCode::RuntimeError};
      }
      sample.position[i] = position->Data()[0];
      sample.velocity[i] = velocity->Data()[0];
    }
    sample.valid_fields = DualJointState::kPositionValid |
                          DualJointState::kVelocityValid;
    sample.sequence = ++sequence_;
    if (!sample.valid()) return {mca::core::StatusCode::NumericalError};
    output = sample;
    return {};
  }

  mca::core::Status write_effort(
      const DualJointEffortCommand& command) noexcept override {
    if (lifecycle_.load() != Lifecycle::Active || safe_stop_requested_.load() || !ecm_) {
      return {mca::core::StatusCode::NotConfigured};
    }
    if (!command.valid() ||
        (has_command_sequence_ && command.sequence <= last_command_sequence_)) {
      return {mca::core::StatusCode::InvalidArgument};
    }
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      if (!ecm_->SetComponentData<gz::sim::components::JointForceCmd>(
              joints_[i], {command.effort[i]})) {
        return {mca::core::StatusCode::RuntimeError};
      }
    }
    last_command_sequence_ = command.sequence;
    has_command_sequence_ = true;
    return {};
  }

  mca::core::Status request_safe_stop() noexcept override {
    if (lifecycle_.load() != Lifecycle::Active) {
      return {mca::core::StatusCode::NotConfigured};
    }
    safe_stop_requested_.store(true);
    return {};
  }

 private:
  bool apply_zero_effort() noexcept {
    if (!ecm_) return false;
    bool success = true;
    for (const auto joint : joints_) {
      success = ecm_->SetComponentData<gz::sim::components::JointForceCmd>(
                    joint, {0.0}) && success;
    }
    return success;
  }

  std::array<gz::sim::Entity, 2> joints_{gz::sim::kNullEntity,
                                         gz::sim::kNullEntity};
  gz::sim::EntityComponentManager* ecm_{nullptr};
  std::atomic<Lifecycle> lifecycle_{Lifecycle::Uninitialized};
  std::atomic<bool> safe_stop_requested_{false};
  std::uint64_t sequence_{0};
  std::uint64_t last_command_sequence_{0};
  bool has_command_sequence_{false};
  bool bound_{false};
};

}  // namespace

GazeboSystem::GazeboSystem() : backend_(create_backend()) {}
GazeboSystem::~GazeboSystem() = default;

std::unique_ptr<IDualJointBackend> GazeboSystem::create_backend() {
  return std::make_unique<GazeboEcmBackend>();
}

GazeboSystem::CallbackReturn GazeboSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  const auto parent = hardware_interface::SystemInterface::on_init(params);
  if (parent != CallbackReturn::SUCCESS) return parent;

  const auto& info = params.hardware_info;
  if (info.type != "system" || info.joints.size() != 2U) {
    return CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const auto& joint = info.joints[i];
    if (joint.name.empty() || joint.command_interfaces.size() != 1U ||
        !HasInterface(joint.command_interfaces, hardware_interface::HW_IF_EFFORT) ||
        !HasInterface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
        !HasInterface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY) ||
        joint.state_interfaces.size() != 2U) {
      return CallbackReturn::ERROR;
    }
    joint_names_[i] = joint.name;
    position_names_[i] = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    velocity_names_[i] = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
    effort_command_names_[i] = joint.name + "/" + hardware_interface::HW_IF_EFFORT;
  }
  if (joint_names_[0] == joint_names_[1]) return CallbackReturn::ERROR;
  return backend_ ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
GazeboSystem::on_export_state_interfaces() {
  auto handles = hardware_interface::SystemInterface::on_export_state_interfaces();
  try {
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      position_handles_[i] = get_state_interface_handle(position_names_[i]);
      velocity_handles_[i] = get_state_interface_handle(velocity_names_[i]);
    }
  } catch (...) {
    position_handles_ = {};
    velocity_handles_ = {};
  }
  return handles;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
GazeboSystem::on_export_command_interfaces() {
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

bool GazeboSystem::initSim(
    rclcpp::Node::SharedPtr&,
    std::map<std::string, gz::sim::Entity>& joints,
    const hardware_interface::HardwareInfo& hardware_info,
    gz::sim::EntityComponentManager& ecm,
    unsigned int) {
  if (!backend_ || hardware_info.joints.size() != joint_names_.size()) return false;
  std::array<gz::sim::Entity, 2> entities{};
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const auto it = joints.find(joint_names_[i]);
    if (it == joints.end() || it->second == gz::sim::kNullEntity) return false;
    entities[i] = it->second;
  }
  return backend_->bind(entities, ecm);
}

GazeboSystem::CallbackReturn GazeboSystem::on_configure(
    const rclcpp_lifecycle::State&) {
  return backend_ && backend_->initialize().ok()
             ? CallbackReturn::SUCCESS
             : CallbackReturn::FAILURE;
}

GazeboSystem::CallbackReturn GazeboSystem::on_activate(
    const rclcpp_lifecycle::State&) {
  if (!backend_ || !backend_->activate().ok()) return CallbackReturn::ERROR;
  active_.store(true);
  has_last_state_sequence_ = false;
  last_state_sequence_ = 0;
  next_command_sequence_ = 0;
  return CallbackReturn::SUCCESS;
}

GazeboSystem::CallbackReturn GazeboSystem::stop_and_deactivate() noexcept {
  if (!backend_ || !active_.load()) return CallbackReturn::SUCCESS;
  if (!backend_->request_safe_stop().ok() || !backend_->deactivate().ok()) {
    return CallbackReturn::ERROR;
  }
  active_.store(false);
  return CallbackReturn::SUCCESS;
}

GazeboSystem::CallbackReturn GazeboSystem::on_deactivate(
    const rclcpp_lifecycle::State&) {
  return stop_and_deactivate();
}
GazeboSystem::CallbackReturn GazeboSystem::on_shutdown(
    const rclcpp_lifecycle::State&) {
  return stop_and_deactivate();
}
GazeboSystem::CallbackReturn GazeboSystem::on_error(
    const rclcpp_lifecycle::State&) {
  return stop_and_deactivate();
}

hardware_interface::return_type GazeboSystem::read(
    const rclcpp::Time&, const rclcpp::Duration&) {
  if (!backend_) return hardware_interface::return_type::ERROR;
  DualJointState sample;
  if (!backend_->read_state(sample).ok() || !sample.valid() ||
      !sample.has(mca::hardware::JointStateField::Position) ||
      !sample.has(mca::hardware::JointStateField::Velocity) ||
      (has_last_state_sequence_ && sample.sequence <= last_state_sequence_)) {
    return hardware_interface::return_type::ERROR;
  }
  try {
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      if (!position_handles_[i] || !velocity_handles_[i] ||
          !set_state(position_handles_[i], sample.position[i], false) ||
          !set_state(velocity_handles_[i], sample.velocity[i], false)) {
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

hardware_interface::return_type GazeboSystem::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
  if (!backend_ || !active_.load() ||
      next_command_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    return hardware_interface::return_type::ERROR;
  }
  DualJointEffortCommand command;
  command.sequence = next_command_sequence_;
  try {
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

}  // namespace mca_gz_ros2_control_adapter

PLUGINLIB_EXPORT_CLASS(mca_gz_ros2_control_adapter::GazeboSystem,
                       gz_ros2_control::GazeboSimSystemInterface)
