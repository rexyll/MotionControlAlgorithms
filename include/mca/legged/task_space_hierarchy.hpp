#pragma once

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace mca::legged {

struct TaskSpaceLevel {
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd target_velocity;
};

struct TaskSpaceHierarchyInput {
  Eigen::Index variables{0};
  std::vector<TaskSpaceLevel> levels;  // Highest priority first.
  double damping{1e-6};
  double rank_tolerance{1e-9};
};

struct TaskSpaceHierarchyOutput {
  Eigen::VectorXd velocity;
  std::vector<double> residual_norms;
  Eigen::Index total_rank{0};
};

// Sequential damped least-squares task solve with null-space projection.
// This is a practical differential-IK baseline, not an exact lexicographic QP.
class TaskSpaceHierarchy {
 public:
  static bool solve(const TaskSpaceHierarchyInput& input,
                    TaskSpaceHierarchyOutput& output,
                    std::string* error = nullptr) {
    const auto fail = [error](const char* message) {
      if (error) *error = message;
      return false;
    };
    if (input.variables <= 0) return fail("task hierarchy variable count must be positive");
    if (input.levels.empty()) return fail("task hierarchy requires at least one level");
    if (!std::isfinite(input.damping) || input.damping < 0.0)
      return fail("task hierarchy damping must be finite and non-negative");
    if (!std::isfinite(input.rank_tolerance) || input.rank_tolerance <= 0.0)
      return fail("task hierarchy rank tolerance must be positive and finite");

    Eigen::VectorXd qdot = Eigen::VectorXd::Zero(input.variables);
    Eigen::MatrixXd projector = Eigen::MatrixXd::Identity(input.variables, input.variables);
    TaskSpaceHierarchyOutput candidate;
    candidate.residual_norms.reserve(input.levels.size());
    for (const auto& level : input.levels) {
      if (level.jacobian.rows() <= 0 || level.jacobian.cols() != input.variables ||
          level.target_velocity.size() != level.jacobian.rows())
        return fail("task hierarchy level dimensions are inconsistent");
      if (!level.jacobian.allFinite() || !level.target_velocity.allFinite())
        return fail("task hierarchy inputs must be finite");

      const Eigen::MatrixXd effective = level.jacobian * projector;
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(effective, Eigen::ComputeThinU | Eigen::ComputeThinV);
      if (svd.info() != Eigen::Success || !svd.singularValues().allFinite())
        return fail("task hierarchy SVD failed");
      Eigen::VectorXd inverse = svd.singularValues();
      Eigen::Index rank = 0;
      for (Eigen::Index i = 0; i < inverse.size(); ++i) {
        const double sigma = svd.singularValues()[i];
        if (sigma > input.rank_tolerance) {
          inverse[i] = sigma / (sigma * sigma + input.damping * input.damping);
          ++rank;
        } else {
          inverse[i] = 0.0;
        }
      }
      const Eigen::MatrixXd pinv =
          svd.matrixV() * inverse.asDiagonal() * svd.matrixU().transpose();
      const Eigen::VectorXd residual = level.target_velocity - level.jacobian * qdot;
      qdot.noalias() += projector * pinv * residual;
      projector = projector * (Eigen::MatrixXd::Identity(input.variables, input.variables) -
                               pinv * effective);
      // Keep numerical drift bounded; symmetrization is not assumed to make it an exact projector.
      if (!qdot.allFinite() || !projector.allFinite())
        return fail("task hierarchy produced non-finite values");
      candidate.residual_norms.push_back((level.jacobian * qdot - level.target_velocity).norm());
      candidate.total_rank += rank;
    }
    candidate.velocity = std::move(qdot);
    output = std::move(candidate);
    return true;
  }
};

}  // namespace mca::legged
