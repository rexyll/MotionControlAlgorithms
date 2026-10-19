#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "mca/transform/clarke_park.hpp"

namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTol = 1e-12;

TEST(ClarkePark, BalancedThreePhaseProducesExpectedAlphaBeta) {
  const auto x = mca::transform::clarke(1.0, -0.5, -0.5);
  EXPECT_NEAR(x.alpha, 1.0, kTol);
  EXPECT_NEAR(x.beta, 0.0, kTol);
  EXPECT_NEAR(x.zero, 0.0, kTol);
}

TEST(ClarkePark, ClarkeRoundTripPreservesAbcIncludingZeroSequence) {
  const double a = 1.2;
  const double b = -0.7;
  const double c = 0.25;
  const auto x = mca::transform::clarke(a, b, c);
  const auto abc = mca::transform::inverse_clarke(x);
  EXPECT_NEAR(abc[0], a, kTol);
  EXPECT_NEAR(abc[1], b, kTol);
  EXPECT_NEAR(abc[2], c, kTol);
}

TEST(ClarkePark, BalancedRotatingVectorHasExpectedAlphaBetaAtSeveralAngles) {
  constexpr std::array<double, 5> angles{
      0.0, kPi / 6.0, kPi / 3.0, kPi / 2.0, kPi};
  for (const double theta : angles) {
    const double a = std::cos(theta);
    const double b = std::cos(theta - 2.0 * kPi / 3.0);
    const double c = std::cos(theta + 2.0 * kPi / 3.0);
    const auto x = mca::transform::clarke(a, b, c);
    EXPECT_NEAR(x.alpha, std::cos(theta), kTol);
    EXPECT_NEAR(x.beta, std::sin(theta), kTol);
    EXPECT_NEAR(x.zero, 0.0, kTol);
  }
}

TEST(ClarkePark, ParkAlignsAlphaBetaVectorWithRotorFrame) {
  constexpr std::array<double, 5> angles{
      0.0, kPi / 6.0, kPi / 3.0, kPi / 2.0, -kPi / 4.0};
  for (const double theta : angles) {
    const auto dq = mca::transform::park(std::cos(theta), std::sin(theta), theta);
    EXPECT_NEAR(dq.d, 1.0, kTol);
    EXPECT_NEAR(dq.q, 0.0, kTol);
  }
}

TEST(ClarkePark, ParkInverseRoundTripIsLosslessToNumericalPrecision) {
  constexpr std::array<double, 5> angles{
      0.0, kPi / 7.0, -kPi / 3.0, kPi, 7.0 * kPi / 5.0};
  for (const double theta : angles) {
    const mca::transform::AlphaBeta input{1.3, -0.8};
    const auto dq = mca::transform::park(input, theta);
    const auto output = mca::transform::inverse_park(dq, theta);
    EXPECT_NEAR(output.alpha, input.alpha, kTol);
    EXPECT_NEAR(output.beta, input.beta, kTol);
  }
}

TEST(ClarkePark, ParkUsesExplicitPositiveAngleConvention) {
  const auto dq = mca::transform::park(1.0, 0.0, kPi / 2.0);
  EXPECT_NEAR(dq.d, 0.0, kTol);
  EXPECT_NEAR(dq.q, -1.0, kTol);
}

}  // namespace
