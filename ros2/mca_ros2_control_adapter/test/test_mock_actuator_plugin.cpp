#include <memory>

#include <gtest/gtest.h>
#include "hardware_interface/actuator_interface.hpp"
#include "pluginlib/class_loader.hpp"

TEST(McaRos2ControlAdapter, MockActuatorPluginIsDiscoverableAndConstructible) {
  pluginlib::ClassLoader<hardware_interface::ActuatorInterface> loader(
      "hardware_interface", "hardware_interface::ActuatorInterface");

  ASSERT_TRUE(loader.isClassAvailable("mca_ros2_control_adapter/MockActuator"));
  const auto actuator =
      loader.createSharedInstance("mca_ros2_control_adapter/MockActuator");
  ASSERT_NE(actuator, nullptr);
}
