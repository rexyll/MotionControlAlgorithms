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
      // These Gazebo components wrap one value per joint axis. Creating them
      // with their default constructor leaves an empty vector until the first
      // physics update; ros2_control may call read() before that update and
      // immediately transition the hardware to ERROR. Seed the single-axis
      // components with a valid zero sample/command instead.
      const auto ensure_single_axis_component = [this, joint](auto component_tag) {
        using Component = decltype(component_tag);
        const auto* existing = ecm_->Component<Component>(joint);
        if (!existing) {
          ecm_->CreateComponent(joint, Component({0.0}));
        } else if (existing->Data().empty()) {
          if (!ecm_->SetComponentData<Component>(joint, {0.0})) return false;
        }
        const auto* initialized = ecm_->Component<Component>(joint);
        return initialized && initialized->Data().size() == 1U;
      };

      if (!ensure_single_axis_component(gz::sim::components::JointPosition()) ||
          !ensure_single_axis_component(gz::sim::components::JointVelocity()) ||
          !ensure_single_axis_component(gz::sim::components::JointForceCmd())) {
        return false;
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
      // ECM returns false when the value is unchanged, not only on failure.
      // A zero command is commonly identical to the seeded startup value.
      ecm_->SetComponentData<gz::sim::components::JointForceCmd>(
          joints_[i], {command.effort[i]});
      const auto* applied =
          ecm_->Component<gz::sim::components::JointForceCmd>(joints_[i]);
      if (!applied || applied->Data().size() != 1U ||
          applied->Data()[0] != command.effort[i]) {
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
      ecm_->SetComponentData<gz::sim::components::JointForceCmd>(joint, {0.0});
      const auto* applied =
          ecm_->Component<gz::sim::components::JointForceCmd>(joint);
      success = (applied && applied->Data().size() == 1U &&
                 applied->Data()[0] == 0.0) && success;
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
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    auto handle = get_command_interface_handle(effort_command_names_[i]);
    // ros2_control initializes an unspecified double command interface to NaN.
    // Seed effort commands with a safe zero before the first controller update.
    if (!handle || !set_command(handle, 0.0, true)) {
      RCLCPP_ERROR(get_logger(),
                   "GazeboSystem could not initialize zero command for '%s'",
                   effort_command_names_[i].c_str());
      effort_command_handles_ = {};
      return {};
    }
    effort_command_handles_[i] = std::move(handle);
  }
  return handles;
}

bool GazeboSystem::initSim(
    rclcpp::Node::SharedPtr& model_nh,
    std::map<std::string, gz::sim::Entity>& joints,
    const hardware_interface::HardwareInfo& hardware_info,
    gz::sim::EntityComponentManager& ecm,
    unsigned int) {
  // gz_ros2_control calls initSim() before ResourceManager::import_component(),
  // and import_component() is what invokes on_init(). Do not read joint_names_
  // here: it is intentionally populated later by on_init().
  if (!backend_ || hardware_info.joints.size() != joint_names_.size()) {
    RCLCPP_ERROR(model_nh->get_logger(),
                 "GazeboSystem initSim: expected exactly %zu URDF joints, got %zu",
                 joint_names_.size(), hardware_info.joints.size());
    return false;
  }

  std::array<gz::sim::Entity, 2> entities{};
  for (std::size_t i = 0; i < hardware_info.joints.size(); ++i) {
    const auto& joint_name = hardware_info.joints[i].name;
    const auto it = joints.find(joint_name);
    if (it == joints.end()) {
      RCLCPP_ERROR(model_nh->get_logger(),
                   "GazeboSystem initSim: URDF joint '%s' is absent from Gazebo model; "
                   "Gazebo exposes %zu enabled joints",
                   joint_name.c_str(), joints.size());
      for (const auto& [available_name, entity] : joints) {
        RCLCPP_ERROR(model_nh->get_logger(), "  Gazebo joint '%s' entity=%lu",
                     available_name.c_str(), static_cast<unsigned long>(entity));
      }
      return false;
    }
    if (it->second == gz::sim::kNullEntity) {
      RCLCPP_ERROR(model_nh->get_logger(),
                   "GazeboSystem initSim: URDF joint '%s' resolved to a null Gazebo entity",
                   joint_name.c_str());
      return false;
    }
    entities[i] = it->second;
    RCLCPP_INFO(model_nh->get_logger(),
                "GazeboSystem initSim: resolved URDF joint '%s' to Gazebo entity=%lu",
                joint_name.c_str(), static_cast<unsigned long>(it->second));
  }

  if (!backend_->bind(entities, ecm)) {
    RCLCPP_ERROR(model_nh->get_logger(),
                 "GazeboSystem initSim: backend rejected Gazebo entities [%lu, %lu]",
                 static_cast<unsigned long>(entities[0]),
                 static_cast<unsigned long>(entities[1]));
    return false;
  }
  RCLCPP_INFO(model_nh->get_logger(),
              "GazeboSystem initSim: successfully bound %zu Gazebo joints",
              entities.size());
  return true;
}

GazeboSystem::CallbackReturn GazeboSystem::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!backend_) {
    RCLCPP_ERROR(get_logger(), "GazeboSystem on_configure failed: backend is null");
    return CallbackReturn::FAILURE;
  }
  if (!backend_->initialize().ok()) {
    RCLCPP_ERROR(get_logger(),
                 "GazeboSystem on_configure failed: backend initialize rejected lifecycle state");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(), "GazeboSystem on_configure succeeded");
  return CallbackReturn::SUCCESS;
}

GazeboSystem::CallbackReturn GazeboSystem::on_activate(
    const rclcpp_lifecycle::State&) {
  if (!backend_) {
    RCLCPP_ERROR(get_logger(), "GazeboSystem on_activate failed: backend is null");
    return CallbackReturn::ERROR;
  }
  if (!backend_->activate().ok()) {
    RCLCPP_ERROR(get_logger(),
                 "GazeboSystem on_activate failed: backend rejected activation");
    return CallbackReturn::ERROR;
  }
  active_.store(true);
  has_last_state_sequence_ = false;
  last_state_sequence_ = 0;
  next_command_sequence_ = 0;
  RCLCPP_INFO(get_logger(), "GazeboSystem on_activate succeeded");
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
  static std::atomic<bool> read_error_reported{false};
  if (!backend_) {
    if (!read_error_reported.exchange(true)) {
      RCLCPP_ERROR(get_logger(), "GazeboSystem read failed: backend is null");
    }
    return hardware_interface::return_type::ERROR;
  }
  DualJointState sample;
  const auto read_status = backend_->read_state(sample);
  if (!read_status.ok() || !sample.valid() ||
      !sample.has(mca::hardware::JointStateField::Position) ||
      !sample.has(mca::hardware::JointStateField::Velocity) ||
      (has_last_state_sequence_ && sample.sequence <= last_state_sequence_)) {
    if (!read_error_reported.exchange(true)) {
      RCLCPP_ERROR(get_logger(),
                   "GazeboSystem read failed: backend_status_ok=%s sample_valid=%s "
                   "position_valid=%s velocity_valid=%s sequence=%lu previous_sequence=%lu",
                   read_status.ok() ? "true" : "false",
                   sample.valid() ? "true" : "false",
                   sample.has(mca::hardware::JointStateField::Position) ? "true" : "false",
                   sample.has(mca::hardware::JointStateField::Velocity) ? "true" : "false",
                   static_cast<unsigned long>(sample.sequence),
                   static_cast<unsigned long>(last_state_sequence_));
    }
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
  const auto write_status = backend_->write_effort(command);
  if (!command.valid() || !write_status.ok()) {
    static std::atomic<bool> write_error_reported{false};
    if (!write_error_reported.exchange(true)) {
      RCLCPP_ERROR(get_logger(),
                   "GazeboSystem write failed: command_valid=%s backend_status_ok=%s",
                   command.valid() ? "true" : "false",
                   write_status.ok() ? "true" : "false");
    }
    return hardware_interface::return_type::ERROR;
  }
  ++next_command_sequence_;
  return hardware_interface::return_type::OK;
}

}  // namespace mca_gz_ros2_control_adapter

PLUGINLIB_EXPORT_CLASS(mca_gz_ros2_control_adapter::GazeboSystem,
                       gz_ros2_control::GazeboSimSystemInterface)
