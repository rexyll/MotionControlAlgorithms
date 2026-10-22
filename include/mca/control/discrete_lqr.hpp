#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <Eigen/Dense>

namespace mca::control {
struct DiscreteLqrOptions {
  int max_iterations{10000};
  double convergence_tolerance{1e-10};
  double symmetry_tolerance{1e-9};
};
struct DiscreteLqrResult {
  Eigen::MatrixXd K;
  Eigen::MatrixXd P;
  int iterations{0};
  double residual{0.0};
};

// Solve the DARE for x+ = A*x + B*u and cost sum(x'Qx + u'Ru),
// returning u = -K*x. Q should be positive semidefinite and R positive definite.
inline bool solve_discrete_lqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, DiscreteLqrResult& output,
    const DiscreteLqrOptions& options = {}, std::string* error = nullptr) {
  const auto fail = [error](const char* message) { if (error) *error = message; return false; };
  if (A.rows() <= 0 || A.rows() != A.cols() || B.rows() != A.rows() || B.cols() <= 0 ||
      Q.rows() != A.rows() || Q.cols() != A.cols() || R.rows() != B.cols() || R.cols() != B.cols())
    return fail("incompatible or empty matrix dimensions");
  if (!A.allFinite() || !B.allFinite() || !Q.allFinite() || !R.allFinite())
    return fail("matrices must contain finite values");
  if (options.max_iterations <= 0 || !std::isfinite(options.convergence_tolerance) ||
      options.convergence_tolerance <= 0.0 || !std::isfinite(options.symmetry_tolerance) ||
      options.symmetry_tolerance < 0.0)
    return fail("invalid Riccati iteration options");
  if ((Q-Q.transpose()).norm() > options.symmetry_tolerance ||
      (R-R.transpose()).norm() > options.symmetry_tolerance)
    return fail("Q and R must be symmetric");
  Eigen::LLT<Eigen::MatrixXd> r_factor(R);
  if (r_factor.info() != Eigen::Success) return fail("R must be positive definite");
  Eigen::MatrixXd P = 0.5*(Q+Q.transpose());
  const Eigen::MatrixXd Qs = P;
  const Eigen::MatrixXd Rs = 0.5*(R+R.transpose());
  DiscreteLqrResult candidate;
  bool converged = false;
  for (int i=0; i<options.max_iterations; ++i) {
    const Eigen::MatrixXd S = Rs + B.transpose()*P*B;
    Eigen::LDLT<Eigen::MatrixXd> factor(S);
    if (factor.info() != Eigen::Success || !factor.isPositive())
      return fail("R + B'PB is not positive definite");
    const Eigen::MatrixXd K = factor.solve(B.transpose()*P*A);
    if (!K.allFinite()) return fail("Riccati gain contains non-finite values");
    Eigen::MatrixXd next = Qs + A.transpose()*P*A - A.transpose()*P*B*K;
    next = 0.5*(next+next.transpose());
    if (!next.allFinite()) return fail("Riccati iteration produced non-finite values");
    const double residual = (next-P).norm();
    P = std::move(next);
    candidate.iterations = i+1;
    candidate.residual = residual;
    if (residual <= options.convergence_tolerance*(1.0+P.norm())) {
      converged = true;
      break;
    }
  }
  if (!converged) return fail("Riccati iteration did not converge");
  const Eigen::MatrixXd S = Rs + B.transpose()*P*B;
  Eigen::LDLT<Eigen::MatrixXd> factor(S);
  if (factor.info() != Eigen::Success || !factor.isPositive()) return fail("final control Hessian is not positive definite");
  candidate.K = factor.solve(B.transpose()*P*A);
  candidate.P = P;
  if (!candidate.K.allFinite() || !candidate.P.allFinite()) return fail("LQR solution contains non-finite values");
  output = std::move(candidate);
  if (error) error->clear();
  return true;
}
} // namespace mca::control
