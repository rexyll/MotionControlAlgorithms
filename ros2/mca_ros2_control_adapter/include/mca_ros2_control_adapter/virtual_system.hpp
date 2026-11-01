#pragma once

#include "mca_ros2_control_adapter/system_hardware.hpp"

namespace mca_ros2_control_adapter {

// Two-joint deterministic plant backed by MCA's vendor-neutral virtual device.
// Simulation only; this plugin performs no physical hardware I/O.
class VirtualSystem final : public SystemHardware {
 protected:
  std::unique_ptr<IDualJointBackend> create_backend() override;
};

}  // namespace mca_ros2_control_adapter
