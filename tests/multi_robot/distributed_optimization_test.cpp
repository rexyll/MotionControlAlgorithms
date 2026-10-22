#include "mca/multi_robot/distributed_optimization.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using mca::multi_robot::DistributedOptimizationInput;
using mca::multi_robot::DistributedOptimizationOutput;
using mca::multi_robot::distributed_optimization_step;

DistributedOptimizationInput input_for_test() {
  DistributedOptimizationInput input;
  input.estimates.resize(2, 1);
  input.estimates << 0.0, 2.0;
  input.gradients.resize(2, 1);
  input.gradients << 1.0, 3.0;
  input.gradients_at_next_estimates.resize(2, 1);
  input.gradients_at_next_estimates << 0.5, 2.5;
  input.gradient_trackers.resize(2, 1);
  input.gradient_trackers << 1.0, 3.0;
  input.mixing.resize(2, 2);
  input.mixing << 0.5, 0.5, 0.5, 0.5;
  input.step_size = 0.1;
  return input;
}

TEST(DistributedOptimization, AppliesMixingAndGradientStep) {
  const auto input = input_for_test();
  DistributedOptimizationOutput output;
  ASSERT_TRUE(distributed_optimization_step(input, output));
  EXPECT_NEAR(output.estimates(0, 0), 0.9, 1e-12);
  EXPECT_NEAR(output.estimates(1, 0), 0.7, 1e-12);
  EXPECT_NEAR(output.gradient_trackers(0, 0), 1.5, 1e-12);
  EXPECT_NEAR(output.gradient_trackers(1, 0), 1.5, 1e-12);
}

TEST(DistributedOptimization, SupportsMultipleCoordinates) {
  auto input = input_for_test();
  input.estimates.resize(2, 2);
  input.estimates << 0.0, 1.0, 2.0, 3.0;
  input.gradients = Eigen::MatrixXd::Ones(2, 2);
  input.gradients_at_next_estimates = Eigen::MatrixXd::Constant(2, 2, 2.0);
  input.gradient_trackers = Eigen::MatrixXd::Zero(2, 2);
  DistributedOptimizationOutput output;
  ASSERT_TRUE(distributed_optimization_step(input, output));
  EXPECT_NEAR(output.estimates(0, 0), 1.0, 1e-12);
  EXPECT_NEAR(output.estimates(0, 1), 2.0, 1e-12);
  EXPECT_NEAR(output.gradient_trackers(1, 1), 1.0, 1e-12);
}

TEST(DistributedOptimization, RejectsNonDoublyStochasticMixing) {
  auto input = input_for_test();
  input.mixing << 0.8, 0.2, 0.4, 0.6;
  DistributedOptimizationOutput output;
  EXPECT_FALSE(distributed_optimization_step(input, output));
}

TEST(DistributedOptimization, RejectsInvalidInputsTransactionally) {
  auto input = input_for_test();
  DistributedOptimizationOutput output;
  output.estimates = Eigen::MatrixXd::Constant(1, 1, 17.0);
  output.gradient_trackers = Eigen::MatrixXd::Constant(1, 1, 19.0);
  input.step_size = 0.0;
  std::string error;
  EXPECT_FALSE(distributed_optimization_step(input, output, &error));
  EXPECT_FALSE(error.empty());
  ASSERT_EQ(output.estimates.rows(), 1);
  EXPECT_DOUBLE_EQ(output.estimates(0, 0), 17.0);
  EXPECT_DOUBLE_EQ(output.gradient_trackers(0, 0), 19.0);
  input = input_for_test();
  input.estimates(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(distributed_optimization_step(input, output));
}
}  // namespace
