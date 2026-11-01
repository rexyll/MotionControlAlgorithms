#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "mca/hardware/interfaces.hpp"

namespace mca_ros2_control_adapter {

class IScalarJointBackend : public mca::hardware::IHardwareLifecycle,
                            public mca::hardware::IJointStateSource<double, 1>,
                            public mca::hardware::IJointEffortCommandSink<double, 1> {
 public:
  ~IScalarJointBackend() override = default;
};

class ActuatorHardware : public hardware_interface::ActuatorInterface {
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
  virtual std::unique_ptr<IScalarJointBackend> create_backend() = 0;

 private:
  bool validate_backend_state(const mca::hardware::ScalarJointState& state) const noexcept;
  CallbackReturn stop_and_deactivate() noexcept;

  std::unique_ptr<IScalarJointBackend> backend_;
  std::string joint_name_;
  std::string position_state_interface_name_;
  std::string velocity_state_interface_name_;
  std::string acceleration_state_interface_name_;
  std::string effort_state_interface_name_;
  std::string effort_command_interface_name_;

  hardware_interface::StateInterface::SharedPtr position_state_handle_;
  hardware_interface::StateInterface::SharedPtr velocity_state_handle_;
  hardware_interface::StateInterface::SharedPtr acceleration_state_handle_;
  hardware_interface::StateInterface::SharedPtr effort_state_handle_;
  hardware_interface::CommandInterface::SharedPtr effort_command_handle_;

  std::atomic<bool> active_{false};
  bool has_acceleration_state_{false};
  bool has_effort_state_{false};
  bool has_last_state_sequence_{false};
  std::uint64_t last_state_sequence_{0};
  std::uint64_t next_command_sequence_{0};
};

}  // namespace mca_ros2_control_adapter
