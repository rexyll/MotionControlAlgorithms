#pragma once
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include "mca/simulation/ros2_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mca_ros2_adapter {
class ScalarAdapter final : public mca::simulation::IScalarRos2SimulationAdapter {
 public:
  using Clock = std::chrono::steady_clock;
  ScalarAdapter(rclcpp::Node::SharedPtr node, std::string state_topic,
                std::string command_topic,
                std::chrono::milliseconds command_timeout = std::chrono::milliseconds(250))
      : node_(std::move(node)), command_timeout_(command_timeout) {
    if (!node_) throw std::invalid_argument("ScalarAdapter requires a valid ROS 2 node");
    if (command_timeout_.count() <= 0)
      throw std::invalid_argument("command timeout must be positive");
    state_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        std::move(state_topic), rclcpp::QoS(10));
    command_subscription_ = node_->create_subscription<std_msgs::msg::Float64>(
        std::move(command_topic), rclcpp::QoS(10),
        [this](std_msgs::msg::Float64::ConstSharedPtr message) {
          if (!message || !std::isfinite(message->data)) return;
          std::lock_guard<std::mutex> lock(command_mutex_);
          latest_command_ = message->data;
          command_received_at_ = Clock::now();
        });
  }
  bool publish_state(const mca::simulation::SimulationStep& step,
                     const State& state) noexcept override {
    if (!step.valid() || !state.finite()) return false;
    try {
      std_msgs::msg::Float64MultiArray message;
      message.data = {step.time_s, step.dt_s, static_cast<double>(step.step_index),
                      state.position[0], state.velocity[0], state.acceleration[0]};
      state_publisher_->publish(message);
      return true;
    } catch (...) { return false; }
  }
  bool read_command(Command& output) noexcept override {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!latest_command_) return false;
    const auto age = Clock::now() - command_received_at_;
    if (age < Clock::duration::zero() ||
        age > std::chrono::duration_cast<Clock::duration>(command_timeout_)) return false;
    Command candidate{};
    candidate.effort[0] = *latest_command_;
    if (!candidate.finite()) return false;
    output = candidate;
    return true;
  }
 private:
  rclcpp::Node::SharedPtr node_;
  std::chrono::milliseconds command_timeout_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_subscription_;
  std::mutex command_mutex_;
  std::optional<double> latest_command_;
  Clock::time_point command_received_at_{};
};
}  // namespace mca_ros2_adapter
