#pragma once

#include <Eigen/Core>

#include "mca/core/types.hpp"

namespace mca::control {

struct ControllerConfig {
  double update_period_s{0.001};
};

struct ControlState {
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd effort;
};

struct ControlReference {
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd effort;
};

struct ControlContext {
  core::Time time{};
  double dt{0.0};
};

struct ControlCommand {
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd effort;
};

class IController {
 public:
  virtual ~IController() = default;
  virtual core::Status configure(const ControllerConfig& config) noexcept = 0;
  virtual core::Status reset(const ControlState& state) noexcept = 0;
  virtual core::Status update(const ControlState& state,
                              const ControlReference& reference,
                              const ControlContext& context,
                              ControlCommand& command) noexcept = 0;
};

class IPositionController : public IController {
 public:
  ~IPositionController() override = default;
};

class IVelocityController : public IController {
 public:
  ~IVelocityController() override = default;
};

class ITorqueController : public IController {
 public:
  ~ITorqueController() override = default;
};

}  // namespace mca::control
