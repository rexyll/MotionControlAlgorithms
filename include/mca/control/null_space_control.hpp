#pragma once

#include <cmath>
#include <string>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/SVD>

namespace mca::control {

// Computes a joint-space null-space projector from a task Jacobian.
// The Jacobian maps joint velocity to task velocity. With zero damping,
// the Moore-Penrose pseudoinverse is truncated using singular_tolerance.
// Positive damping uses the damped inverse sigma/(sigma^2 + damping^2).
class NullSpaceControl {
 public:
  static bool compute(const Eigen::MatrixXd& jacobian,
                      const Eigen::VectorXd& secondary_velocity,
                      Eigen::MatrixXd& projector,
                      Eigen::VectorXd& nullspace_velocity,
                      double damping = 0.0,
                      double singular_tolerance = 1e-9,
                      std::string* error = nullptr) {
    if (jacobian.rows() <= 0 || jacobian.cols() <= 0 ||
        !jacobian.allFinite())
      return fail("Null-space Jacobian must be non-empty and finite", error);
    if (secondary_velocity.size() != jacobian.cols() ||
        !secondary_velocity.allFinite())
      return fail("Null-space secondary velocity must match Jacobian columns and be finite", error);
    if (!std::isfinite(damping) || damping < 0.0)
      return fail("Null-space damping must be finite and non-negative", error);
    if (!std::isfinite(singular_tolerance) || singular_tolerance <= 0.0)
      return fail("Null-space singular tolerance must be positive and finite", error);

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (svd.info() != Eigen::Success || !svd.singularValues().allFinite())
      return fail("Null-space Jacobian SVD failed", error);

    const Eigen::VectorXd singular_values = svd.singularValues();
    Eigen::VectorXd inverse_values(singular_values.size());
    for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
      const double sigma = singular_values[i];
      if (damping > 0.0) {
        inverse_values[i] = sigma / (sigma * sigma + damping * damping);
      } else {
        inverse_values[i] = sigma > singular_tolerance ? 1.0 / sigma : 0.0;
      }
    }

    const Eigen::MatrixXd pseudo_inverse =
        svd.matrixV() * inverse_values.asDiagonal() * svd.matrixU().transpose();
    Eigen::MatrixXd candidate_projector =
        Eigen::MatrixXd::Identity(jacobian.cols(), jacobian.cols()) -
        pseudo_inverse * jacobian;
    Eigen::VectorXd candidate_velocity = candidate_projector * secondary_velocity;
    if (!candidate_projector.allFinite() || !candidate_velocity.allFinite())
      return fail("Null-space computation produced non-finite output", error);

    projector = std::move(candidate_projector);
    nullspace_velocity = std::move(candidate_velocity);
    return true;
  }

 private:
  static bool fail(const char* message, std::string* error) {
    if (error) *error = message;
    return false;
  }
};

}  // namespace mca::control
