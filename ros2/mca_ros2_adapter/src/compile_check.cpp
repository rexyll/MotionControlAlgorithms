#include "mca_ros2_adapter/scalar_adapter.hpp"

namespace mca_ros2_adapter {
void compile_check(rclcpp::Node::SharedPtr node) {
  ScalarAdapter adapter(std::move(node), "mca/state", "mca/command");
  mca::simulation::SimulationStep step{0.0, 0.001, 0};
  mca::simulation::ScalarSimulationState state{};
  (void)adapter.publish_state(step, state);
  mca::simulation::ScalarActuatorCommand command{};
  (void)adapter.read_command(command);
}
}  // namespace mca_ros2_adapter
