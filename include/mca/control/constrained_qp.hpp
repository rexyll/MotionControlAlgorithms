#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>

#include <Eigen/Dense>

namespace mca::control {

struct ConstrainedQpOptions {
  std::size_t max_iterations{200U};
  double rho{1.0};
  double absolute_tolerance{1e-7};
  double relative_tolerance{1e-6};
  double symmetry_tolerance{1e-9};
  double positive_semidefinite_tolerance{1e-9};
};

struct ConstrainedQpProblem {
  Eigen::MatrixXd hessian;
  Eigen::VectorXd gradient;
  Eigen::MatrixXd equality_matrix;
  Eigen::VectorXd equality_rhs;
  Eigen::VectorXd lower_bound;
  Eigen::VectorXd upper_bound;
};

struct ConstrainedQpResult {
  Eigen::VectorXd solution;
  double objective{0.0};
  double primal_residual{0.0};
  double dual_residual{0.0};
  std::size_t iterations{0U};
};

namespace detail {
inline bool finite_matrix(const Eigen::MatrixXd& value) {
  return value.array().isFinite().all();
}

inline bool finite_vector(const Eigen::VectorXd& value) {
  return value.array().isFinite().all();
}
}  // namespace detail

// Solves a convex quadratic program
//   min 0.5*x' H x + g' x
//   s.t. A x = b, lower_bound <= x <= upper_bound.
// The implementation uses ADMM with an exact equality-constrained x-update and
// a box projection. It is a deterministic, dependency-free baseline for
// control applications rather than a high-performance production solver.
inline bool solve_constrained_qp(const ConstrainedQpProblem& problem,
                                 const ConstrainedQpOptions& options,
                                 ConstrainedQpResult& output,
                                 std::string* error = nullptr) {
  const auto fail = [&](const char* message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };

  const Eigen::Index n = problem.gradient.size();
  if (n <= 0) {
    return fail("gradient must be non-empty");
  }
  if (problem.hessian.rows() != n || problem.hessian.cols() != n) {
    return fail("hessian dimensions must match gradient");
  }
  if (problem.lower_bound.size() != n || problem.upper_bound.size() != n) {
    return fail("bounds dimensions must match gradient");
  }
  const Eigen::Index m = problem.equality_matrix.rows();
  if (problem.equality_matrix.cols() != n || problem.equality_rhs.size() != m) {
    return fail("equality dimensions are inconsistent");
  }
  if (!detail::finite_matrix(problem.hessian) || !detail::finite_vector(problem.gradient) ||
      !detail::finite_matrix(problem.equality_matrix) || !detail::finite_vector(problem.equality_rhs) ||
      !detail::finite_vector(problem.lower_bound) || !detail::finite_vector(problem.upper_bound)) {
    return fail("all inputs must be finite");
  }
  if (options.max_iterations == 0U || !std::isfinite(options.rho) || options.rho <= 0.0 ||
      !std::isfinite(options.absolute_tolerance) || options.absolute_tolerance <= 0.0 ||
      !std::isfinite(options.relative_tolerance) || options.relative_tolerance <= 0.0 ||
      !std::isfinite(options.symmetry_tolerance) || options.symmetry_tolerance < 0.0 ||
      !std::isfinite(options.positive_semidefinite_tolerance) ||
      options.positive_semidefinite_tolerance < 0.0) {
    return fail("invalid solver options");
  }
  if ((problem.lower_bound.array() > problem.upper_bound.array()).any()) {
    return fail("lower bound exceeds upper bound");
  }

  const Eigen::MatrixXd h = 0.5 * (problem.hessian + problem.hessian.transpose());
  const double symmetry_error =
      (problem.hessian - problem.hessian.transpose()).cwiseAbs().maxCoeff();
  if (symmetry_error > options.symmetry_tolerance) {
    return fail("hessian must be symmetric within tolerance");
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(h);
  if (eigen_solver.info() != Eigen::Success ||
      eigen_solver.eigenvalues().minCoeff() < -options.positive_semidefinite_tolerance) {
    return fail("hessian must be positive semidefinite");
  }

  if (m > 0) {
    Eigen::FullPivLU<Eigen::MatrixXd> rank_check(problem.equality_matrix);
    if (rank_check.rank() < m) {
      return fail("equality constraints must be linearly independent");
    }
  }

  Eigen::MatrixXd kkt;
  if (m > 0) {
    kkt = Eigen::MatrixXd::Zero(n + m, n + m);
    kkt.topLeftCorner(n, n) = h + options.rho * Eigen::MatrixXd::Identity(n, n);
    kkt.topRightCorner(n, m) = problem.equality_matrix.transpose();
    kkt.bottomLeftCorner(m, n) = problem.equality_matrix;
  }

  Eigen::VectorXd z = Eigen::VectorXd::Zero(n);
  z = z.cwiseMax(problem.lower_bound).cwiseMin(problem.upper_bound);
  Eigen::VectorXd x = z;
  Eigen::VectorXd dual = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd previous_z = z;

  Eigen::LDLT<Eigen::MatrixXd> unconstrained_solver;
  Eigen::FullPivLU<Eigen::MatrixXd> equality_solver;
  if (m == 0) {
    unconstrained_solver.compute(h + options.rho * Eigen::MatrixXd::Identity(n, n));
    if (unconstrained_solver.info() != Eigen::Success) {
      return fail("failed to factor QP Hessian");
    }
  } else {
    equality_solver.compute(kkt);
    if (equality_solver.rank() < n + m) {
      return fail("equality-constrained KKT system is singular");
    }
  }

  bool converged = false;
  std::size_t iterations = 0U;
  for (std::size_t iteration = 0U; iteration < options.max_iterations; ++iteration) {
    const Eigen::VectorXd rhs_x = -problem.gradient + options.rho * (z - dual);
    if (m == 0) {
      x = unconstrained_solver.solve(rhs_x);
    } else {
      Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n + m);
      rhs.head(n) = rhs_x;
      rhs.tail(m) = problem.equality_rhs;
      const Eigen::VectorXd kkt_solution = equality_solver.solve(rhs);
      x = kkt_solution.head(n);
    }
    if (!detail::finite_vector(x)) {
      return fail("solver produced a non-finite iterate");
    }

    previous_z = z;
    z = (x + dual).cwiseMax(problem.lower_bound).cwiseMin(problem.upper_bound);
    dual += x - z;

    const double primal = (x - z).norm();
    const double dual_residual = options.rho * (z - previous_z).norm();
    const double scale = std::max({1.0, x.norm(), z.norm(), previous_z.norm()});
    const double tolerance = options.absolute_tolerance + options.relative_tolerance * scale;
    iterations = iteration + 1U;
    if (primal <= tolerance && dual_residual <= tolerance) {
      converged = true;
      break;
    }
  }

  if (!converged) {
    return fail("QP solver did not converge within the iteration limit");
  }

  const double equality_residual =
      m == 0 ? 0.0 : (problem.equality_matrix * z - problem.equality_rhs).norm();
  const double equality_tolerance = options.absolute_tolerance +
                                    options.relative_tolerance * std::max(1.0, problem.equality_rhs.norm());
  if (equality_residual > equality_tolerance) {
    return fail("equality constraints were not satisfied");
  }

  const double objective = 0.5 * z.dot(h * z) + problem.gradient.dot(z);
  if (!std::isfinite(objective)) {
    return fail("objective is non-finite");
  }

  ConstrainedQpResult candidate;
  candidate.solution = z;
  candidate.objective = objective;
  candidate.primal_residual = (x - z).norm();
  candidate.dual_residual = options.rho * (z - previous_z).norm();
  candidate.iterations = iterations;
  output = std::move(candidate);
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

}  // namespace mca::control
