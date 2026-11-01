#include <memory>
#include <string>

#include <gtest/gtest.h>
#include "gz_ros2_control/gz_system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"

namespace {

hardware_interface::HardwareInfo MakeInfo() {
  hardware_interface::HardwareInfo info;
  info.name = "mca_gazebo_system";
  info.type = "system";
  for (const auto* joint_name : {"joint_a", "joint_b"}) {
    hardware_interface::ComponentInfo joint;
    joint.name = joint_name;
    hardware_interface::InterfaceInfo effort;
    effort.name = hardware_interface::HW_IF_EFFORT;
    joint.command_interfaces.push_back(effort);
    hardware_interface::InterfaceInfo position;
    position.name = hardware_interface::HW_IF_POSITION;
    joint.state_interfaces.push_back(position);
    hardware_interface::InterfaceInfo velocity;
    velocity.name = hardware_interface::HW_IF_VELOCITY;
    joint.state_interfaces.push_back(velocity);
    info.joints.push_back(joint);
  }
  return info;
}

}  // namespace

TEST(McaGazeboRos2ControlAdapter, PluginIsDiscoverableAndConstructible) {
  pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface> loader(
      "gz_ros2_control", "gz_ros2_control::GazeboSimSystemInterface");
  ASSERT_TRUE(loader.isClassAvailable(
      "mca_gz_ros2_control_adapter/GazeboSystem"));
  const auto system =
      loader.createSharedInstance("mca_gz_ros2_control_adapter/GazeboSystem");
  ASSERT_NE(system, nullptr);
}

TEST(McaGazeboRos2ControlAdapter, AcceptsExactlyTwoEffortControlledJoints) {
  pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface> loader(
      "gz_ros2_control", "gz_ros2_control::GazeboSimSystemInterface");
  auto system =
      loader.createSharedInstance("mca_gz_ros2_control_adapter/GazeboSystem");
  ASSERT_NE(system, nullptr);
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = MakeInfo();
  EXPECT_EQ(system->on_init(params), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system->on_export_state_interfaces().size(), 4U);
  EXPECT_EQ(system->on_export_command_interfaces().size(), 2U);
}
