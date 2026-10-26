#include <limits>
#include <memory>
#include <thread>
#include <chrono>

#include "gtest/gtest.h"
#include "mca_ros2_adapter/scalar_adapter.hpp"
#include "std_msgs/msg/float64.hpp"

class ScalarAdapterTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(ScalarAdapterTest, RejectsInvalidStateWithoutPublishing) {
  auto node = std::make_shared<rclcpp::Node>("mca_scalar_adapter_invalid_state_test");
  mca_ros2_adapter::ScalarAdapter adapter(node, "state", "command");
  mca::simulation::ScalarSimulationState state{};
  EXPECT_FALSE(adapter.publish_state({0.0, 0.0, 0}, state));
  state.position[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter.publish_state({0.0, 0.01, 0}, state));
}

TEST_F(ScalarAdapterTest, MissingCommandPreservesCallerOutput) {
  auto node = std::make_shared<rclcpp::Node>("mca_scalar_adapter_missing_command_test");
  mca_ros2_adapter::ScalarAdapter adapter(node, "state", "command");
  mca::simulation::ScalarActuatorCommand output{};
  output.effort[0] = 12.5;
  EXPECT_FALSE(adapter.read_command(output));
  EXPECT_DOUBLE_EQ(output.effort[0], 12.5);
}

TEST_F(ScalarAdapterTest, ReadsFiniteCommandFromRosTopic) {
  auto node = std::make_shared<rclcpp::Node>("mca_scalar_adapter_command_test");
  mca_ros2_adapter::ScalarAdapter adapter(node, "state", "command");
  auto publisher = node->create_publisher<std_msgs::msg::Float64>("command", rclcpp::QoS(10));
  std_msgs::msg::Float64 message;
  message.data = 3.25;
  publisher->publish(message);
  mca::simulation::ScalarActuatorCommand output{};
  bool received = false;
  for (int attempt = 0; attempt < 30 && !received; ++attempt) {
    rclcpp::spin_some(node);
    received = adapter.read_command(output);
    if (!received) std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(received);
  EXPECT_DOUBLE_EQ(output.effort[0], 3.25);
}
