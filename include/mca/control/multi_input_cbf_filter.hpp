#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <utility>
#include <vector>
#include <Eigen/Dense>
#include "mca/core/types.hpp"

namespace mca::control {
struct MultiInputCbfConfig {
  Eigen::VectorXd control_min;
  Eigen::VectorXd control_max;
  double alpha{1.0};
  double tolerance{1.0e-8};
  std::size_t max_total_constraints{16U};
};
struct MultiInputCbfInput {
  Eigen::VectorXd nominal_control;
  Eigen::VectorXd barrier_values;
  Eigen::VectorXd drift_derivatives;
  Eigen::MatrixXd control_derivatives;
};
struct MultiInputCbfOutput {
  Eigen::VectorXd control;
  double minimum_safety_margin{0.0};
  std::size_t active_constraints{0U};
  bool modified{false};
};

// Small-problem projection for vector controls and multiple first-order CBFs.
class MultiInputCbfFilter {
 public:
  core::Status configure(const MultiInputCbfConfig& config) {
    const Eigen::Index n = config.control_min.size();
    if (n <= 0 || config.control_max.size() != n ||
        !config.control_min.array().isFinite().all() ||
        !config.control_max.array().isFinite().all() ||
        (config.control_min.array() > config.control_max.array()).any() ||
        !core::is_finite(config.alpha) || config.alpha <= 0.0 ||
        !core::is_finite(config.tolerance) || config.tolerance <= 0.0 ||
        config.max_total_constraints == 0U || config.max_total_constraints > 20U)
      return {core::StatusCode::InvalidArgument};
    config_ = config;
    configured_ = true;
    return {};
  }

  core::Status filter(const MultiInputCbfInput& input, MultiInputCbfOutput& output) const {
    if (!configured_) return {core::StatusCode::NotConfigured};
    const Eigen::Index n = config_.control_min.size();
    const Eigen::Index m = input.barrier_values.size();
    if (input.nominal_control.size() != n || m <= 0 ||
        input.drift_derivatives.size() != m || input.control_derivatives.rows() != m ||
        input.control_derivatives.cols() != n)
      return {core::StatusCode::InvalidArgument};
    if (!input.nominal_control.array().isFinite().all() ||
        !input.barrier_values.array().isFinite().all() ||
        !input.drift_derivatives.array().isFinite().all() ||
        !input.control_derivatives.array().isFinite().all())
      return {core::StatusCode::InvalidArgument};

    const Eigen::VectorXd base = input.drift_derivatives + config_.alpha * input.barrier_values;
    if (!base.array().isFinite().all()) return {core::StatusCode::NumericalError};
    const Eigen::Index total = m + 2 * n;
    if (total > static_cast<Eigen::Index>(config_.max_total_constraints))
      return {core::StatusCode::InvalidArgument};

    // Convert G*u + base >= 0 and actuator bounds into C*u >= d.
    Eigen::MatrixXd c = Eigen::MatrixXd::Zero(total, n);
    Eigen::VectorXd d = Eigen::VectorXd::Zero(total);
    c.topRows(m) = input.control_derivatives;
    d.head(m) = -base;
    for (Eigen::Index i = 0; i < n; ++i) {
      c(m + i, i) = 1.0; d[m + i] = config_.control_min[i];
      c(m + n + i, i) = -1.0; d[m + n + i] = -config_.control_max[i];
    }

    bool found = false;
    double best_cost = std::numeric_limits<double>::infinity();
    Eigen::VectorXd best;
    std::vector<Eigen::Index> active;
    const Eigen::Index max_active = std::min(n, total);
    const auto consider = [&]() {
      const Eigen::Index k = static_cast<Eigen::Index>(active.size());
      Eigen::VectorXd candidate;
      if (k == 0) {
        candidate = input.nominal_control;
      } else {
        Eigen::MatrixXd kkt = Eigen::MatrixXd::Zero(n + k, n + k);
        kkt.topLeftCorner(n, n).setIdentity();
        for (Eigen::Index row = 0; row < k; ++row) {
          const Eigen::Index idx = active[static_cast<std::size_t>(row)];
          kkt.block(0, n + row, n, 1) = -c.row(idx).transpose();
          kkt.block(n + row, 0, 1, n) = c.row(idx);
        }
        Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n + k);
        rhs.head(n) = input.nominal_control;
        for (Eigen::Index row = 0; row < k; ++row)
          rhs[n + row] = d[active[static_cast<std::size_t>(row)]];
        Eigen::FullPivLU<Eigen::MatrixXd> solver(kkt);
        if (solver.rank() != n + k) return;
        const Eigen::VectorXd solution = solver.solve(rhs);
        if (!solution.array().isFinite().all()) return;
        for (Eigen::Index row = 0; row < k; ++row)
          if (solution[n + row] < -config_.tolerance) return;
        candidate = solution.head(n);
      }
      const Eigen::VectorXd margins = c * candidate - d;
      if (!candidate.array().isFinite().all() ||
          (margins.array() < -config_.tolerance).any()) return;
      const double cost = (candidate - input.nominal_control).squaredNorm();
      if (cost < best_cost) { best_cost = cost; best = candidate; found = true; }
    };

    // A strictly convex projection has an independent active subset of at most n rows.
    std::function<void(Eigen::Index, Eigen::Index)> enumerate =
      [&](Eigen::Index next, Eigen::Index target_size) {
        if (static_cast<Eigen::Index>(active.size()) == target_size) { consider(); return; }
        const Eigen::Index needed = target_size - static_cast<Eigen::Index>(active.size());
        for (Eigen::Index i = next; i <= total - needed; ++i) {
          active.push_back(i); enumerate(i + 1, target_size); active.pop_back();
        }
      };
    for (Eigen::Index size = 0; size <= max_active; ++size) enumerate(0, size);
    if (!found) return {core::StatusCode::ConstraintViolation};

    const Eigen::VectorXd safety_margins = base + input.control_derivatives * best;
    if (!safety_margins.array().isFinite().all()) return {core::StatusCode::NumericalError};
    MultiInputCbfOutput candidate;
    candidate.control = best;
    candidate.minimum_safety_margin = safety_margins.minCoeff();
    candidate.active_constraints = static_cast<std::size_t>(
      (safety_margins.array().abs() <= 10.0 * config_.tolerance).count());
    candidate.modified = (best - input.nominal_control).norm() > config_.tolerance;
    output = std::move(candidate);
    return {};
  }
  [[nodiscard]] bool configured() const noexcept { return configured_; }
 private:
  MultiInputCbfConfig config_{};
  bool configured_{false};
};
}  // namespace mca::control
