#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>
namespace mca::legged {
struct ContactWrenchOptimizationInput {
  std::vector<Eigen::Vector3d> contact_positions_world;
  Eigen::Matrix<double, 6, 1> desired_wrench_world{Eigen::Matrix<double, 6, 1>::Zero()};
  std::vector<double> friction_coefficients;
  double minimum_normal_force{0.0};
  double maximum_normal_force{1000.0};
  double regularization{1e-6};
  int iterations{250};
  double step_size{0.0};
};
struct ContactWrenchOptimizationOutput {
  Eigen::Matrix<double, 3, Eigen::Dynamic> forces_world;
  Eigen::Matrix<double, 6, 1> achieved_wrench_world{Eigen::Matrix<double, 6, 1>::Zero()};
  double residual_norm{0.0};
  double objective{0.0};
  int iterations{0};
};
// Deterministic projected-gradient force distribution with unilateral normal
// bounds and a conservative square friction pyramid; no contact moments.
class ContactWrenchOptimizer {
 public:
  static bool solve(const ContactWrenchOptimizationInput& input,
                    ContactWrenchOptimizationOutput& output,
                    std::string* error = nullptr) {
    const auto fail = [error](const char* message) {
      if (error) *error = message;
      return false;
    };
    const Eigen::Index count = static_cast<Eigen::Index>(input.contact_positions_world.size());
    if (count <= 0) return fail("at least one contact position is required");
    if (input.friction_coefficients.size() != static_cast<std::size_t>(count))
      return fail("friction coefficient count must match contact count");
    if (!input.desired_wrench_world.allFinite()) return fail("desired wrench must be finite");
    if (!std::isfinite(input.minimum_normal_force) || input.minimum_normal_force < 0.0 ||
        !std::isfinite(input.maximum_normal_force) ||
        input.maximum_normal_force < input.minimum_normal_force)
      return fail("normal-force bounds must be finite, non-negative and ordered");
    if (!std::isfinite(input.regularization) || input.regularization <= 0.0)
      return fail("regularization must be positive and finite");
    if (input.iterations <= 0) return fail("iteration count must be positive");
    if (!std::isfinite(input.step_size)) return fail("step size must be finite");
    for (Eigen::Index i = 0; i < count; ++i) {
      if (!input.contact_positions_world[static_cast<std::size_t>(i)].allFinite())
        return fail("contact positions must be finite");
      const double mu = input.friction_coefficients[static_cast<std::size_t>(i)];
      if (!std::isfinite(mu) || mu < 0.0)
        return fail("friction coefficients must be finite and non-negative");
    }
    const Eigen::Index variables = 3 * count;
    Eigen::MatrixXd map = Eigen::MatrixXd::Zero(6, variables);
    for (Eigen::Index i = 0; i < count; ++i) {
      map.block<3, 3>(0, 3 * i).setIdentity();
      const Eigen::Vector3d& r = input.contact_positions_world[static_cast<std::size_t>(i)];
      Eigen::Matrix3d skew;
      skew << 0.0, -r.z(), r.y(), r.z(), 0.0, -r.x(), -r.y(), r.x(), 0.0;
      map.block<3, 3>(3, 3 * i) = skew;
    }
    const Eigen::MatrixXd hessian = map.transpose() * map +
        input.regularization * Eigen::MatrixXd::Identity(variables, variables);
    const Eigen::VectorXd linear = -map.transpose() * input.desired_wrench_world;
    const double bound = map.squaredNorm() + input.regularization;
    const double step = input.step_size > 0.0 ? input.step_size : 1.0 / bound;
    if (!(step > 0.0) || !std::isfinite(step) || step > 2.0 / bound)
      return fail("step size exceeds the conservative stability bound");
    Eigen::VectorXd forces = Eigen::VectorXd::Zero(variables);
    for (int iteration = 0; iteration < input.iterations; ++iteration) {
      const Eigen::VectorXd gradient = hessian * forces + linear;
      forces.noalias() -= step * gradient;
      for (Eigen::Index i = 0; i < count; ++i) {
        const double mu = input.friction_coefficients[static_cast<std::size_t>(i)];
        const double normal = std::clamp(forces[3 * i + 2],
            input.minimum_normal_force, input.maximum_normal_force);
        const double limit = mu * normal / std::sqrt(2.0);
        forces[3 * i] = std::clamp(forces[3 * i], -limit, limit);
        forces[3 * i + 1] = std::clamp(forces[3 * i + 1], -limit, limit);
        forces[3 * i + 2] = normal;
      }
      if (!forces.allFinite()) return fail("optimizer produced non-finite forces");
    }
    ContactWrenchOptimizationOutput candidate;
    candidate.forces_world.resize(3, count);
    for (Eigen::Index i = 0; i < count; ++i)
      candidate.forces_world.col(i) = forces.segment<3>(3 * i);
    candidate.achieved_wrench_world = map * forces;
    const Eigen::VectorXd residual = candidate.achieved_wrench_world - input.desired_wrench_world;
    candidate.residual_norm = residual.norm();
    candidate.objective = 0.5 * residual.squaredNorm() +
                          0.5 * input.regularization * forces.squaredNorm();
    candidate.iterations = input.iterations;
    if (!candidate.forces_world.allFinite() || !candidate.achieved_wrench_world.allFinite() ||
        !std::isfinite(candidate.residual_norm) || !std::isfinite(candidate.objective))
      return fail("optimizer result is non-finite");
    output = std::move(candidate);
    return true;
  }
};
}  // namespace mca::legged
