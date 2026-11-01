#pragma once

#include "mca_ros2_control_adapter/actuator_hardware.hpp"

namespace mca_ros2_control_adapter {

// Deterministic plugin used to validate the ros2_control boundary. It is not a
// physical device driver and does not simulate plant dynamics.
class MockActuator final : public ActuatorHardware {
 protected:
  std::unique_ptr<IScalarJointBackend> create_backend() override;
};

}  // namespace mca_ros2_control_adapter
