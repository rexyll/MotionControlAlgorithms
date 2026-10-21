#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace mca::legged {

struct QPTaskLevel {
  Eigen::MatrixXd matrix;
  Eigen::VectorXd target;
  double weight{1.0};
};

struct QPTaskHierarchyInput {
  Eigen::Index variables{0};
  std::vector<QPTaskLevel> levels;  // Ordered from highest to lowest priority.
  double regularization{1e-8};
  Eigen::VectorXd lower_bounds;
  Eigen::VectorXd upper_bounds;
  int iterations{80};
  double step_size{0.0};  // <= 0 selects a Lipschitz-based step.
};

struct QPTaskHierarchyOutput {
  Eigen::VectorXd solution;
  double objective{0.0};
  int iterations{0};
};

// Deterministic convex weighted-QP baseline for task hierarchy.
// min 0.5*x' H x + g' x, with optional box constraints.
// Level weights encode priority; this is not an exact lexicographic solver.
class QPTaskHierarchy {
 public:
  static bool solve(const QPTaskHierarchyInput& input,
                    QPTaskHierarchyOutput& output,
                    std::string* error = nullptr) {
    const Eigen::Index n = input.variables;
    if (n <= 0) return fail("QP hierarchy variable count must be positive", error);
    if (input.levels.empty()) return fail("QP hierarchy requires at least one task level", error);
    if (!std::isfinite(input.regularization) || input.regularization <= 0.0)
      return fail("QP hierarchy regularization must be positive and finite", error);
    if (input.iterations <= 0) return fail("QP hierarchy iterations must be positive", error);
    if (input.lower_bounds.size() != 0 && input.lower_bounds.size() != n)
      return fail("QP hierarchy lower bounds have invalid dimension", error);
    if (input.upper_bounds.size() != 0 && input.upper_bounds.size() != n)
      return fail("QP hierarchy upper bounds have invalid dimension", error);
    if (input.lower_bounds.size() != 0 && input.upper_bounds.size() != 0 &&
        (input.lower_bounds.array() > input.upper_bounds.array()).any())
      return fail("QP hierarchy lower bounds exceed upper bounds", error);
    if (input.lower_bounds.size() != 0 && !input.lower_bounds.allFinite())
      return fail("QP hierarchy lower bounds must be finite", error);
    if (input.upper_bounds.size() != 0 && !input.upper_bounds.allFinite())
      return fail("QP hierarchy upper bounds must be finite", error);

    Eigen::MatrixXd H = input.regularization * Eigen::MatrixXd::Identity(n, n);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(n);
    for (const auto& level : input.levels) {
      if (level.matrix.rows() <= 0 || level.matrix.cols() != n ||
          level.target.size() != level.matrix.rows())
        return fail("QP hierarchy task dimensions are inconsistent", error);
      if (!level.matrix.allFinite() || !level.target.allFinite() ||
          !std::isfinite(level.weight) || level.weight <= 0.0)
        return fail("QP hierarchy task inputs must be finite and positive", error);
      H.noalias() += level.weight * level.matrix.transpose() * level.matrix;
      g.noalias() -= level.weight * level.matrix.transpose() * level.target;
    }
    if (!H.allFinite() || !g.allFinite())
      return fail("QP hierarchy assembled non-finite coefficients", error);

    const double lambda_max = H.selfadjointView<Eigen::Upper>().eigenvalues().maxCoeff();
    if (!std::isfinite(lambda_max) || lambda_max <= 0.0)
      return fail("QP hierarchy Hessian is not positive definite", error);
    const double alpha = input.step_size > 0.0 ? input.step_size : 1.0 / lambda_max;
    if (!std::isfinite(alpha) || alpha <= 0.0)
      return fail("QP hierarchy step size must be positive and finite", error);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    if (input.lower_bounds.size() != 0) x = x.cwiseMax(input.lower_bounds);
    if (input.upper_bounds.size() != 0) x = x.cwiseMin(input.upper_bounds);

    for (int k = 0; k < input.iterations; ++k) {
      const Eigen::VectorXd gradient = H * x + g;
      if (!gradient.allFinite()) return fail("QP hierarchy gradient became non-finite", error);
      Eigen::VectorXd next = x - alpha * gradient;
      if (input.lower_bounds.size() != 0) next = next.cwiseMax(input.lower_bounds);
      if (input.upper_bounds.size() != 0) next = next.cwiseMin(input.upper_bounds);
      if (!next.allFinite()) return fail("QP hierarchy iterate became non-finite", error);
      x = std::move(next);
    }

    const double objective = 0.5 * x.dot(H * x) + g.dot(x);
    if (!std::isfinite(objective)) return fail("QP hierarchy objective became non-finite", error);
    output.solution = std::move(x);
    output.objective = objective;
    output.iterations = input.iterations;
    return true;
  }

 private:
  static bool fail(const char* message, std::string* error) {
    if (error) *error = message;
    return false;
  }
};

}  // namespace mca::legged
