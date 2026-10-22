#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "mca/legged/contact_wrench_optimizer.hpp"
namespace {
mca::legged::ContactWrenchOptimizationInput make_input() {
  mca::legged::ContactWrenchOptimizationInput input;
  input.contact_positions_world = {Eigen::Vector3d(-0.25, 0.0, 0.0),
                                   Eigen::Vector3d(0.25, 0.0, 0.0)};
  input.friction_coefficients = {0.6, 0.6};
  input.desired_wrench_world << 0.0, 0.0, 20.0, 0.0, 0.0, 0.0;
  input.iterations = 400;
  return input;
}
}
TEST(ContactWrenchOptimizerTest, DistributesSymmetricVerticalLoad) {
  const auto input = make_input();
  mca::legged::ContactWrenchOptimizationOutput output;
  ASSERT_TRUE(mca::legged::ContactWrenchOptimizer::solve(input, output));
  ASSERT_EQ(output.forces_world.cols(), 2);
  EXPECT_NEAR(output.forces_world(2, 0), 10.0, 0.1);
  EXPECT_NEAR(output.forces_world(2, 1), 10.0, 0.1);
  EXPECT_NEAR(output.achieved_wrench_world[2], 20.0, 0.1);
  EXPECT_LT(output.residual_norm, 0.1);
  EXPECT_EQ(output.iterations, 400);
}
TEST(ContactWrenchOptimizerTest, EnforcesUnilateralAndFrictionBounds) {
  auto input = make_input();
  input.desired_wrench_world << 100.0, -100.0, 40.0, 0.0, 0.0, 0.0;
  mca::legged::ContactWrenchOptimizationOutput output;
  ASSERT_TRUE(mca::legged::ContactWrenchOptimizer::solve(input, output));
  for (Eigen::Index i = 0; i < output.forces_world.cols(); ++i) {
    const double fx = output.forces_world(0, i), fy = output.forces_world(1, i);
    const double fz = output.forces_world(2, i);
    EXPECT_GE(fz, input.minimum_normal_force);
    EXPECT_LE(fz, input.maximum_normal_force);
    EXPECT_LE(std::abs(fx), input.friction_coefficients[static_cast<std::size_t>(i)] * fz / std::sqrt(2.0) + 1e-10);
    EXPECT_LE(std::abs(fy), input.friction_coefficients[static_cast<std::size_t>(i)] * fz / std::sqrt(2.0) + 1e-10);
  }
}
TEST(ContactWrenchOptimizerTest, RejectsInvalidInputTransactionally) {
  auto input = make_input();
  input.friction_coefficients[0] = -1.0;
  mca::legged::ContactWrenchOptimizationOutput output;
  output.residual_norm = 123.0;
  output.iterations = 17;
  EXPECT_FALSE(mca::legged::ContactWrenchOptimizer::solve(input, output));
  EXPECT_DOUBLE_EQ(output.residual_norm, 123.0);
  EXPECT_EQ(output.iterations, 17);
  input = make_input();
  input.contact_positions_world[0].x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mca::legged::ContactWrenchOptimizer::solve(input, output));
  EXPECT_DOUBLE_EQ(output.residual_norm, 123.0);
}
