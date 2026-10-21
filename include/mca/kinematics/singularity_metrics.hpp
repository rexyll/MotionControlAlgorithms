#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>
#include "mca/kinematics/jacobian.hpp"

namespace mca::kinematics {

struct SingularityMetricsResult {
  std::vector<double> singular_values{};  // descending order
  std::size_t numerical_rank{0};
  double minimum_singular_value{0.0};
  double maximum_singular_value{0.0};
  double condition_number{std::numeric_limits<double>::infinity()};
  double manipulability{0.0};
  bool singular{true};
};

struct SingularityMetricsOptions {
  double relative_rank_tolerance{1e-9};
  double absolute_rank_tolerance{1e-12};
  std::size_t max_eigen_iterations{128};
};

class SingularityMetrics {
 public:
  static bool solve(const JacobianResult& jacobian, SingularityMetricsResult& out,
                    const SingularityMetricsOptions& options = {},
                    std::string* error = nullptr) {
    if (!std::isfinite(options.relative_rank_tolerance) ||
        options.relative_rank_tolerance < 0.0 ||
        !std::isfinite(options.absolute_rank_tolerance) ||
        options.absolute_rank_tolerance < 0.0 ||
        options.max_eigen_iterations == 0U) {
      return fail("invalid singularity metric options", error);
    }
    if (jacobian.columns == 0U) return fail("Jacobian has no columns", error);
    for (const auto& row : jacobian.rows) {
      if (row.size() != jacobian.columns) return fail("Jacobian row size mismatch", error);
      for (double value : row) {
        if (!std::isfinite(value)) return fail("Jacobian contains a non-finite value", error);
      }
    }

    // Work with the smaller Gram matrix to limit computation to at most 6x6.
    const std::size_t n = std::min<std::size_t>(6U, jacobian.columns);
    std::array<std::array<double, 6>, 6> gram{};
    if (jacobian.columns <= 6U) {
      for (std::size_t r = 0; r < n; ++r)
        for (std::size_t c = 0; c < n; ++c)
          for (std::size_t k = 0; k < 6U; ++k)
            gram[r][c] += jacobian.rows[k][r] * jacobian.rows[k][c];
    } else {
      for (std::size_t r = 0; r < 6U; ++r)
        for (std::size_t c = 0; c < 6U; ++c)
          for (std::size_t k = 0; k < jacobian.columns; ++k)
            gram[r][c] += jacobian.rows[r][k] * jacobian.rows[c][k];
    }

    // Jacobi rotations for the eigenvalues of a real symmetric Gram matrix.
    for (std::size_t iteration = 0; iteration < options.max_eigen_iterations; ++iteration) {
      std::size_t p = 0U, q = 0U;
      double largest = 0.0;
      for (std::size_t r = 0; r < n; ++r) {
        for (std::size_t c = r + 1U; c < n; ++c) {
          if (std::abs(gram[r][c]) > largest) {
            largest = std::abs(gram[r][c]); p = r; q = c;
          }
        }
      }
      double scale = 1.0;
      for (std::size_t i = 0; i < n; ++i) scale = std::max(scale, std::abs(gram[i][i]));
      if (largest <= 1e-14 * scale) break;
      const double tau = (gram[q][q] - gram[p][p]) / (2.0 * gram[p][q]);
      const double t = (tau >= 0.0 ? 1.0 : -1.0) /
                       (std::abs(tau) + std::sqrt(1.0 + tau * tau));
      const double cs = 1.0 / std::sqrt(1.0 + t * t);
      const double sn = t * cs;
      const double app = gram[p][p], aqq = gram[q][q], apq = gram[p][q];
      gram[p][p] = app - t * apq;
      gram[q][q] = aqq + t * apq;
      gram[p][q] = 0.0; gram[q][p] = 0.0;
      for (std::size_t k = 0; k < n; ++k) {
        if (k == p || k == q) continue;
        const double akp = gram[k][p], akq = gram[k][q];
        gram[k][p] = cs * akp - sn * akq; gram[p][k] = gram[k][p];
        gram[k][q] = sn * akp + cs * akq; gram[q][k] = gram[k][q];
      }
    }

    std::vector<double> singular_values;
    singular_values.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      if (!std::isfinite(gram[i][i])) return fail("eigendecomposition produced a non-finite value", error);
      singular_values.push_back(std::sqrt(std::max(0.0, gram[i][i])));
    }
    std::sort(singular_values.begin(), singular_values.end(), std::greater<double>());

    SingularityMetricsResult candidate;
    candidate.singular_values = singular_values;
    candidate.maximum_singular_value = singular_values.empty() ? 0.0 : singular_values.front();
    candidate.minimum_singular_value = singular_values.empty() ? 0.0 : singular_values.back();
    const double threshold = std::max(options.absolute_rank_tolerance,
                                      options.relative_rank_tolerance * candidate.maximum_singular_value);
    candidate.manipulability = 1.0;
    for (double sigma : singular_values) {
      if (sigma > threshold) ++candidate.numerical_rank;
      candidate.manipulability *= sigma;
    }
    if (singular_values.empty()) candidate.manipulability = 0.0;
    candidate.singular = candidate.numerical_rank < n;
    if (!candidate.singular && candidate.minimum_singular_value > 0.0)
      candidate.condition_number = candidate.maximum_singular_value / candidate.minimum_singular_value;
    else {
      candidate.condition_number = std::numeric_limits<double>::infinity();
      candidate.manipulability = 0.0;
    }
    if (!std::isfinite(candidate.manipulability)) return fail("manipulability overflowed", error);
    out = std::move(candidate);
    return true;
  }

 private:
  static bool fail(const char* message, std::string* error) {
    if (error) *error = message;
    return false;
  }
};

}  // namespace mca::kinematics
