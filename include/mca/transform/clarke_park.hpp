#pragma once

#include <array>
#include <cmath>

namespace mca::transform {

struct AlphaBeta {
  double alpha{0.0};
  double beta{0.0};
};

struct Dq {
  double d{0.0};
  double q{0.0};
};

struct AlphaBetaZero {
  double alpha{0.0};
  double beta{0.0};
  double zero{0.0};
};

// Amplitude-invariant Clarke transform.
// abc -> alpha-beta-zero preserves the peak amplitude of a balanced
// sinusoidal set. The zero-sequence component is the arithmetic mean.
inline AlphaBetaZero clarke(double a, double b, double c) noexcept {
  constexpr double sqrt3 = 1.7320508075688772935274463415059;
  constexpr double two_over_three = 2.0 / 3.0;
  return {
      two_over_three * (a - 0.5 * b - 0.5 * c),
      two_over_three * (sqrt3 * 0.5) * (b - c),
      (a + b + c) / 3.0};
}

inline std::array<double, 3> inverse_clarke(const AlphaBetaZero& x) noexcept {
  constexpr double sqrt3 = 1.7320508075688772935274463415059;
  return {
      x.alpha + x.zero,
      -0.5 * x.alpha + (sqrt3 * 0.5) * x.beta + x.zero,
      -0.5 * x.alpha - (sqrt3 * 0.5) * x.beta + x.zero};
}

// Park transform using the convention
//   d = alpha*cos(theta) + beta*sin(theta)
//   q = -alpha*sin(theta) + beta*cos(theta)
// A positive theta rotates the stationary alpha-beta frame into the rotor
// frame. This convention is used consistently by inverse_park below.
inline Dq park(const AlphaBeta& x, double electrical_angle) noexcept {
  const double c = std::cos(electrical_angle);
  const double s = std::sin(electrical_angle);
  return {x.alpha * c + x.beta * s,
          -x.alpha * s + x.beta * c};
}

inline AlphaBeta inverse_park(const Dq& x, double electrical_angle) noexcept {
  const double c = std::cos(electrical_angle);
  const double s = std::sin(electrical_angle);
  return {x.d * c - x.q * s,
          x.d * s + x.q * c};
}

inline Dq park(double alpha, double beta, double electrical_angle) noexcept {
  return park(AlphaBeta{alpha, beta}, electrical_angle);
}

inline AlphaBeta inverse_park(double d, double q,
                              double electrical_angle) noexcept {
  return inverse_park(Dq{d, q}, electrical_angle);
}

}  // namespace mca::transform
