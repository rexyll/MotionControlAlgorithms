#pragma once

#include "mca_ros2_control_adapter/system_hardware.hpp"

namespace mca_ros2_control_adapter {

// Deterministic two-joint test backend; no plant dynamics or physical I/O.
class MockSystem final : public SystemHardware {
 protected:
  std::unique_ptr<IDualJointBackend> create_backend() override;
};

}  // namespace mca_ros2_control_adapter
