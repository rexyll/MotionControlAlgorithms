
#include <cmath>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"

namespace {

hardware_interface::HardwareInfo MakeActuatorInfo() {
  hardware_interface::HardwareInfo info;
  info.name = "mca_mock_actuator";
  info.type = "actuator";

  hardware_interface::ComponentInfo joint;
  joint.name = "motor_joint";

  hardware_interface::InterfaceInfo effort_command;
  effort_command.name = hardware_interface::HW_IF_EFFORT;
  joint.command_interfaces.push_back(effort_command);

  hardware_interface::InterfaceInfo position_state;
  position_state.name = hardware_interface::HW_IF_POSITION;
  joint.state_interfaces.push_back(position_state);

  hardware_interface::InterfaceInfo velocity_state;
  velocity_state.name = hardware_interface::HW_IF_VELOCITY;
  joint.state_interfaces.push_back(velocity_state);

  info.joints.push_back(joint);
  return info;
}

}  // namespace

TEST(McaRos2ControlAdapter, MockActuatorPluginIsDiscoverableAndConstructible) {
  pluginlib::ClassLoader<hardware_interface::ActuatorInterface> loader(
      "hardware_interface", "hardware_interface::ActuatorInterface");

  ASSERT_TRUE(loader.isClassAvailable("mca_ros2_control_adapter/MockActuator"));
  const auto actuator =
      loader.createSharedInstance("mca_ros2_control_adapter/MockActuator");
  ASSERT_NE(actuator, nullptr);
}

TEST(McaRos2ControlAdapter, LifecycleReadWriteAndSafeStopMapToMcaPorts) {
  pluginlib::ClassLoader<hardware_interface::ActuatorInterface> loader(
      "hardware_interface", "hardware_interface::ActuatorInterface");
  auto actuator =
      loader.createSharedInstance("mca_ros2_control_adapter/MockActuator");
  ASSERT_NE(actuator, nullptr);

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = MakeActuatorInfo();
  ASSERT_EQ(actuator->on_init(params),
            hardware_interface::CallbackReturn::SUCCESS);

  const auto state_handles = actuator->on_export_state_interfaces();
  const auto command_handles = actuator->on_export_command_interfaces();
  ASSERT_EQ(state_handles.size(), 2U);
  ASSERT_EQ(command_handles.size(), 1U);

  const rclcpp_lifecycle::State previous_state;
  ASSERT_EQ(actuator->on_configure(previous_state),
            hardware_interface::CallbackReturn::SUCCESS);

  EXPECT_EQ(actuator->read(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);

  double position = 1.0;
  double velocity = 1.0;
  ASSERT_TRUE(state_handles[0]->get_value(position, false));
  ASSERT_TRUE(state_handles[1]->get_value(velocity, false));
  EXPECT_DOUBLE_EQ(position, 0.0);
  EXPECT_DOUBLE_EQ(velocity, 0.0);

  ASSERT_EQ(actuator->on_activate(previous_state),
            hardware_interface::CallbackReturn::SUCCESS);

  ASSERT_TRUE(command_handles[0]->set_value(1.25, false));
  EXPECT_EQ(actuator->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);

  ASSERT_TRUE(command_handles[0]->set_value(
      std::numeric_limits<double>::quiet_NaN(), false));
  EXPECT_EQ(actuator->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::ERROR);

  ASSERT_TRUE(command_handles[0]->set_value(0.0, false));
  EXPECT_EQ(actuator->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);

  EXPECT_EQ(actuator->on_deactivate(previous_state),
            hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(actuator->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::ERROR);
}
