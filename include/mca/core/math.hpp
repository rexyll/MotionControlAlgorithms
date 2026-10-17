#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace mca::math {

constexpr double clamp(double value, double lower, double upper) noexcept {
  return std::min(std::max(value, lower), upper);
}

constexpr double lerp(double a, double b, double t) noexcept {
  return a + (b - a) * t;
}

inline double wrap_angle(double angle) noexcept {
  constexpr double pi = 3.1415926535897932384626433832795;
  constexpr double two_pi = 2.0 * pi;
  angle = std::fmod(angle + pi, two_pi);
  if (angle < 0.0) angle += two_pi;
  return angle - pi;
}

inline bool is_nearly_equal(double a, double b, double abs_tol = 1e-12,
                            double rel_tol = 1e-9) noexcept {
  const double diff = std::abs(a - b);
  if (diff <= abs_tol) return true;
  return diff <= rel_tol * std::max(std::abs(a), std::abs(b));
}

inline bool is_finite(double value) noexcept {
  return std::isfinite(value);
}

}  // namespace mca::math
