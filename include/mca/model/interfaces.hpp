#pragma once

#include <Eigen/Core>

#include "mca/core/types.hpp"

namespace mca::model {

class IKinematicsModel {
 public:
  virtual ~IKinematicsModel() = default;
  virtual core::Status forward(const Eigen::VectorXd& q,
                               Eigen::Matrix4d& pose) const noexcept = 0;
  virtual core::Status jacobian(const Eigen::VectorXd& q,
                                Eigen::MatrixXd& jacobian) const noexcept = 0;
};

class IDynamicsModel {
 public:
  virtual ~IDynamicsModel() = default;
  virtual core::Status mass_matrix(const Eigen::VectorXd& q,
                                   Eigen::MatrixXd& mass) const noexcept = 0;
  virtual core::Status bias_forces(const Eigen::VectorXd& q,
                                   const Eigen::VectorXd& dq,
                                   Eigen::VectorXd& bias) const noexcept = 0;
  virtual core::Status gravity(const Eigen::VectorXd& q,
                               Eigen::VectorXd& gravity) const noexcept = 0;
};

class ILinearModel {
 public:
  virtual ~ILinearModel() = default;
  virtual core::Status matrices(Eigen::MatrixXd& a, Eigen::MatrixXd& b,
                                Eigen::MatrixXd& c, Eigen::MatrixXd& d) const noexcept = 0;
};

}  // namespace mca::model
