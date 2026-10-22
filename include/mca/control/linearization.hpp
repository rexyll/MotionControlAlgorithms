#pragma once
#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <utility>
#include <Eigen/Dense>
namespace mca::control {
using DiscreteDynamicsEvaluator = std::function<bool(const Eigen::VectorXd&, const Eigen::VectorXd&, Eigen::VectorXd&)>;
struct LinearizationOptions { double finite_difference_step{1e-6}; };
struct LinearizationResult {
  Eigen::MatrixXd A, B;
  Eigen::VectorXd affine_offset, operating_state, operating_input, operating_next_state;
};
// Locally approximate x[k+1] = f(x[k],u[k]) by A*x + B*u + affine_offset.
// The evaluator must be deterministic and side-effect-free during this call.
inline bool linearize_discrete_dynamics(const DiscreteDynamicsEvaluator& evaluate,
    const Eigen::VectorXd& state, const Eigen::VectorXd& input,
    LinearizationResult& output, const LinearizationOptions& options = {},
    std::string* error = nullptr) {
  const auto fail = [error](const char* message) { if (error) *error = message; return false; };
  if (!evaluate) return fail("dynamics evaluator is empty");
  if (state.size() <= 0 || input.size() <= 0) return fail("state and input dimensions must be positive");
  if (!state.allFinite() || !input.allFinite()) return fail("operating state and input must be finite");
  if (!std::isfinite(options.finite_difference_step) || options.finite_difference_step <= 0.0)
    return fail("finite_difference_step must be finite and positive");
  Eigen::VectorXd f0;
  if (!evaluate(state, input, f0)) return fail("dynamics evaluator failed at operating point");
  if (f0.size() != state.size()) return fail("next-state dimension must equal state dimension");
  if (!f0.allFinite()) return fail("dynamics output contains non-finite values");
  LinearizationResult candidate;
  candidate.A.resize(state.size(), state.size()); candidate.B.resize(state.size(), input.size());
  candidate.operating_state = state; candidate.operating_input = input; candidate.operating_next_state = f0;
  for (Eigen::Index col = 0; col < state.size(); ++col) {
    const double h = options.finite_difference_step * std::max(1.0, std::abs(state(col)));
    Eigen::VectorXd xp=state, xm=state; xp(col)+=h; xm(col)-=h;
    Eigen::VectorXd fp, fm;
    if (!evaluate(xp,input,fp) || !evaluate(xm,input,fm)) return fail("dynamics evaluator failed during state perturbation");
    if (fp.size()!=state.size() || fm.size()!=state.size()) return fail("perturbed next-state dimension must equal state dimension");
    if (!fp.allFinite() || !fm.allFinite()) return fail("perturbed dynamics output contains non-finite values");
    candidate.A.col(col)=(fp-fm)/(2.0*h);
  }
  for (Eigen::Index col = 0; col < input.size(); ++col) {
    const double h = options.finite_difference_step * std::max(1.0, std::abs(input(col)));
    Eigen::VectorXd up=input, um=input; up(col)+=h; um(col)-=h;
    Eigen::VectorXd fp, fm;
    if (!evaluate(state,up,fp) || !evaluate(state,um,fm)) return fail("dynamics evaluator failed during input perturbation");
    if (fp.size()!=state.size() || fm.size()!=state.size()) return fail("perturbed next-state dimension must equal state dimension");
    if (!fp.allFinite() || !fm.allFinite()) return fail("perturbed dynamics output contains non-finite values");
    candidate.B.col(col)=(fp-fm)/(2.0*h);
  }
  candidate.affine_offset=f0-candidate.A*state-candidate.B*input;
  if (!candidate.A.allFinite() || !candidate.B.allFinite() || !candidate.affine_offset.allFinite())
    return fail("linearization contains non-finite values");
  output=std::move(candidate); if (error) error->clear(); return true;
}
} // namespace mca::control
