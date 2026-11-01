#include <algorithm>
#include <cmath>
#include <string>
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

template <typename Handles>
auto FindHandle(const Handles& handles, const std::string& name)
    -> typename Handles::value_type {
  const auto it = std::find_if(handles.begin(), handles.end(),
                               [&name](const auto& handle) {
                                 return handle && handle->get_name() == name;
                               });
  return it == handles.end() ? typename Handles::value_type{} : *it;
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

  // ros2_control does not promise a stable vector order for exported handles.
  // Resolve every handle by its fully qualified interface name.
  const auto position_a = FindHandle(state_handles, "joint_a/position");
  const auto velocity_a = FindHandle(state_handles, "joint_a/velocity");
  const auto effort_a = FindHandle(state_handles, "joint_a/effort");
  const auto position_b = FindHandle(state_handles, "joint_b/position");
  const auto velocity_b = FindHandle(state_handles, "joint_b/velocity");
  const auto effort_b = FindHandle(state_handles, "joint_b/effort");
  const auto command_a = FindHandle(command_handles, "joint_a/effort");
  const auto command_b = FindHandle(command_handles, "joint_b/effort");
  ASSERT_NE(position_a, nullptr);
  ASSERT_NE(velocity_a, nullptr);
  ASSERT_NE(effort_a, nullptr);
  ASSERT_NE(position_b, nullptr);
  ASSERT_NE(velocity_b, nullptr);
  ASSERT_NE(effort_b, nullptr);
  ASSERT_NE(command_a, nullptr);
  ASSERT_NE(command_b, nullptr);

  const rclcpp_lifecycle::State previous_state;
  ASSERT_EQ(system->on_configure(previous_state),
            hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);

  double value = 0.0;
  ASSERT_TRUE(position_a->get_value(value, false));
  EXPECT_DOUBLE_EQ(value, 0.25);
  ASSERT_TRUE(velocity_a->get_value(value, false));
  EXPECT_DOUBLE_EQ(value, 1.0);
  ASSERT_TRUE(position_b->get_value(value, false));
  EXPECT_DOUBLE_EQ(value, -0.5);

  ASSERT_EQ(system->on_activate(previous_state),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(command_a->set_value(1.25, false));
  ASSERT_TRUE(command_b->set_value(-2.5, false));
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);

  // A non-finite value in either joint rejects the entire command before the
  // backend is called; the backend retains the previously accepted pair.
  ASSERT_TRUE(command_b->set_value(
      std::numeric_limits<double>::quiet_NaN(), false));
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);
  ASSERT_TRUE(effort_a->get_value(value, false));
  EXPECT_DOUBLE_EQ(value, 1.25);
  ASSERT_TRUE(effort_b->get_value(value, false));
  EXPECT_DOUBLE_EQ(value, -2.5);

  ASSERT_TRUE(command_a->set_value(
      std::numeric_limits<double>::infinity(), false));
  ASSERT_TRUE(command_b->set_value(0.0, false));
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::ERROR);

  ASSERT_TRUE(command_a->set_value(0.0, false));
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);

  EXPECT_EQ(system->on_deactivate(previous_state),
            hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);
  ASSERT_TRUE(effort_a->get_value(value, false));
  EXPECT_DOUBLE_EQ(value, 0.0);
  ASSERT_TRUE(effort_b->get_value(value, false));
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
