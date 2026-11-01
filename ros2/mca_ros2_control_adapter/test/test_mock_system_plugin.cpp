#include <cmath>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"

namespace {

hardware_interface::HardwareInfo MakeSystemInfo() {
  hardware_interface::HardwareInfo info;
  info.name = "mca_mock_system";
  info.type = "system";

  for (const auto* name : {"joint_a", "joint_b"}) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;

    hardware_interface::InterfaceInfo effort_command;
    effort_command.name = hardware_interface::HW_IF_EFFORT;
    joint.command_interfaces.push_back(effort_command);

    for (const auto* interface_name : {hardware_interface::HW_IF_POSITION,
                                       hardware_interface::HW_IF_VELOCITY,
                                       hardware_interface::HW_IF_EFFORT}) {
      hardware_interface::InterfaceInfo state;
      state.name = interface_name;
      joint.state_interfaces.push_back(state);
    }
    info.joints.push_back(joint);
  }
  return info;
}

}  // namespace

TEST(McaRos2ControlAdapter, MockSystemPluginIsDiscoverableAndConstructible) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");

  ASSERT_TRUE(loader.isClassAvailable("mca_ros2_control_adapter/MockSystem"));
  const auto system =
      loader.createSharedInstance("mca_ros2_control_adapter/MockSystem");
  ASSERT_NE(system, nullptr);
}

TEST(McaRos2ControlAdapter, MultiJointLifecycleReadWriteAndSafeStop) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  auto system = loader.createSharedInstance("mca_ros2_control_adapter/MockSystem");
  ASSERT_NE(system, nullptr);

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = MakeSystemInfo();
  ASSERT_EQ(system->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  const auto state_handles = system->on_export_state_interfaces();
  const auto command_handles = system->on_export_command_interfaces();
  ASSERT_EQ(state_handles.size(), 6U);
  ASSERT_EQ(command_handles.size(), 2U);

  const rclcpp_lifecycle::State previous_state;
  ASSERT_EQ(system->on_configure(previous_state),
            hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);

  double value = 0.0;
  ASSERT_TRUE(state_handles[0]->get_value(value, false));
  EXPECT_DOUBLE_EQ(value, 0.25);
  ASSERT_TRUE(state_handles[1]->get_value(value, false));
  EXPECT_DOUBLE_EQ(value, 1.0);
  ASSERT_TRUE(state_handles[3]->get_value(value, false));
  EXPECT_DOUBLE_EQ(value, -0.5);

  ASSERT_EQ(system->on_activate(previous_state),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(command_handles[0]->set_value(1.25, false));
  ASSERT_TRUE(command_handles[1]->set_value(-2.5, false));
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);

  // A non-finite value in either joint rejects the entire command before the
  // backend is called; the backend retains the previously accepted pair.
  ASSERT_TRUE(command_handles[1]->set_value(
      std::numeric_limits<double>::quiet_NaN(), false));
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);
  ASSERT_TRUE(state_handles[2]->get_value(value, false));
  EXPECT_DOUBLE_EQ(value, 1.25);
  ASSERT_TRUE(state_handles[5]->get_value(value, false));
  EXPECT_DOUBLE_EQ(value, -2.5);

  ASSERT_TRUE(command_handles[0]->set_value(
      std::numeric_limits<double>::infinity(), false));
  ASSERT_TRUE(command_handles[1]->set_value(0.0, false));
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::ERROR);

  ASSERT_TRUE(command_handles[0]->set_value(0.0, false));
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);

  EXPECT_EQ(system->on_deactivate(previous_state),
            hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);
  ASSERT_TRUE(state_handles[2]->get_value(value, false));
  EXPECT_DOUBLE_EQ(value, 0.0);
  ASSERT_TRUE(state_handles[5]->get_value(value, false));
  EXPECT_DOUBLE_EQ(value, 0.0);
}

TEST(McaRos2ControlAdapter, MockSystemRejectsInvalidJointTopology) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  auto system = loader.createSharedInstance("mca_ros2_control_adapter/MockSystem");
  ASSERT_NE(system, nullptr);

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = MakeSystemInfo();
  params.hardware_info.joints.pop_back();
  EXPECT_EQ(system->on_init(params), hardware_interface::CallbackReturn::ERROR);
}
