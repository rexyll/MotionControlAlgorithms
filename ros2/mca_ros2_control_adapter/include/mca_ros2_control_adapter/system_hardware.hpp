#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "mca/hardware/interfaces.hpp"

namespace mca_ros2_control_adapter {

using DualJointState = mca::hardware::JointState<double, 2>;
using DualJointEffortCommand = mca::hardware::JointEffortCommand<double, 2>;

class IDualJointBackend : public mca::hardware::IHardwareLifecycle,
                          public mca::hardware::IJointStateSource<double, 2>,
                          public mca::hardware::IJointEffortCommandSink<double, 2> {
 public:
  ~IDualJointBackend() override = default;
};

// Fixed two-joint reference SystemInterface. ROS interface storage is created
// during initialization/export; read/write only use pre-cached handles.
class SystemHardware : public hardware_interface::SystemInterface {
 public:
  using CallbackReturn = hardware_interface::CallbackReturn;

  CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                         const rclcpp::Duration& period) override;

 protected:
  virtual std::unique_ptr<IDualJointBackend> create_backend() = 0;

 private:
  CallbackReturn stop_and_deactivate() noexcept;
  bool validate_backend_state(const DualJointState& state) const noexcept;

  std::unique_ptr<IDualJointBackend> backend_;
  std::array<std::string, 2> joint_names_{};
  std::array<std::string, 2> position_names_{};
  std::array<std::string, 2> velocity_names_{};
  std::array<std::string, 2> effort_state_names_{};
  std::array<std::string, 2> effort_command_names_{};

  std::array<hardware_interface::StateInterface::SharedPtr, 2> position_handles_{};
  std::array<hardware_interface::StateInterface::SharedPtr, 2> velocity_handles_{};
  std::array<hardware_interface::StateInterface::SharedPtr, 2> effort_state_handles_{};
  std::array<hardware_interface::CommandInterface::SharedPtr, 2> effort_command_handles_{};

  std::atomic<bool> active_{false};
  bool has_effort_state_{false};
  bool has_last_state_sequence_{false};
  std::uint64_t last_state_sequence_{0};
  std::uint64_t next_command_sequence_{0};
};

}  // namespace mca_ros2_control_adapter
