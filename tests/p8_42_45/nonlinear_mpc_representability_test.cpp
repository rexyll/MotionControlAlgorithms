#include <cmath>
#include <limits>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "mca/control/nonlinear_mpc.hpp"

namespace {
using namespace mca::control;

NonlinearMpcProblem make_problem(double lower, double upper,
                                 const std::function<double(double)>& cost) {
  NonlinearMpcProblem p;
  p.control_lower = Eigen::VectorXd::Constant(1, lower);
  p.control_upper = Eigen::VectorXd::Constant(1, upper);
  p.dynamics = [](const Eigen::VectorXd& x, const Eigen::VectorXd&,
                  Eigen::VectorXd& next) {
    next = x;
    return true;
  };
  p.stage_cost = [cost](const Eigen::VectorXd&, const Eigen::VectorXd& u) {
    return cost(u(0));
  };
  p.terminal_cost = [](const Eigen::VectorXd&) { return 0.0; };
  return p;
}

TEST(NonlinearMpcP84245, LargeControlKeepsRepresentableGradientAndDescent) {
  constexpr double base = 1.0e16;
  auto p = make_problem(0.0, 2.0e16,
                        [](double u) { return -(u - 1.0e16); });
  NonlinearMpcOptions o;
  o.horizon = 1;
  o.optimization_iterations = 1;
  o.finite_difference_step = 0.1;  // Smaller than one ULP at base.
  o.gradient_step = 0.25;           // Update rounds back to base without nextafter.
  o.initial_control_sequence = Eigen::MatrixXd::Constant(1, 1, base);

  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p, Eigen::VectorXd::Zero(1), o, r));
  EXPECT_NEAR(r.projected_gradient_norm, 1.0, 1e-12);
  EXPECT_GT(r.control(0), base);
  EXPECT_LT(r.cost, 0.0);
  EXPECT_EQ(r.line_search_evaluations, 1U);
}

TEST(NonlinearMpcP843, UnderflowedStepGradientCanStillAcceptRepresentableDescent) {
  constexpr double base = 1.0e16;
  auto p = make_problem(0.0, 2.0e16,
                        [](double u) { return -1.0e-200 * (u - 1.0e16); });
  NonlinearMpcOptions o;
  o.horizon = 1;
  o.optimization_iterations = 1;
  o.finite_difference_step = 0.1;
  o.gradient_step = 1.0e-200;
  o.initial_control_sequence = Eigen::MatrixXd::Constant(1, 1, base);

  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p, Eigen::VectorXd::Zero(1), o, r));
  EXPECT_GT(r.control(0), base);
  EXPECT_LT(r.cost, 0.0);
  EXPECT_EQ(r.line_search_evaluations, 1U);
}

TEST(NonlinearMpcP845, UsesFullOneSidedDifferenceNearUpperBound) {
  auto p = make_problem(-1.0, 1.0, [](double u) { return u * u * u; });
  NonlinearMpcOptions o;
  o.horizon = 1;
  o.optimization_iterations = 1;
  o.finite_difference_step = 0.25;
  o.gradient_step = 0.01;
  o.initial_control_sequence = Eigen::MatrixXd::Constant(1, 1, 0.9);

  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p, Eigen::VectorXd::Zero(1), o, r));
  // Backward one-sided quotient: (0.9^3 - 0.65^3) / (0.9 - 0.65).
  const double expected = (0.9 * 0.9 * 0.9 - 0.65 * 0.65 * 0.65) / 0.25;
  EXPECT_NEAR(r.projected_gradient_norm, expected, 1e-12);
  EXPECT_LT(r.control(0), 0.9);
}

}  // namespace
