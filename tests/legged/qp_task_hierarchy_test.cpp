#include "mca/legged/qp_task_hierarchy.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using mca::legged::QPTaskHierarchy;
using mca::legged::QPTaskHierarchyInput;
using mca::legged::QPTaskHierarchyOutput;
using mca::legged::QPTaskLevel;

TEST(QPTaskHierarchy, SolvesWeightedScalarTask) {
  QPTaskHierarchyInput input;
  input.variables = 1;
  QPTaskLevel level;
  level.matrix = Eigen::MatrixXd::Constant(1, 1, 1.0);
  level.target = Eigen::VectorXd::Constant(1, 3.0);
  level.weight = 10.0;
  input.levels.push_back(level);
  input.regularization = 1e-8;
  input.iterations = 200;
  QPTaskHierarchyOutput output;
  ASSERT_TRUE(QPTaskHierarchy::solve(input, output));
  ASSERT_EQ(output.solution.size(), 1);
  EXPECT_NEAR(output.solution[0], 3.0, 1e-6);
  EXPECT_TRUE(std::isfinite(output.objective));
}

TEST(QPTaskHierarchy, RespectsBoxConstraints) {
  QPTaskHierarchyInput input;
  input.variables = 2;
  QPTaskLevel level;
  level.matrix = Eigen::Matrix2d::Identity();
  level.target = Eigen::Vector2d(4.0, -5.0);
  input.levels.push_back(level);
  input.lower_bounds = Eigen::Vector2d(-2.0, -2.0);
  input.upper_bounds = Eigen::Vector2d(2.0, 2.0);
  input.iterations = 100;
  QPTaskHierarchyOutput output;
  ASSERT_TRUE(QPTaskHierarchy::solve(input, output));
  EXPECT_NEAR(output.solution[0], 2.0, 1e-6);
  EXPECT_NEAR(output.solution[1], -2.0, 1e-6);
}

TEST(QPTaskHierarchy, RejectsInvalidInputTransactionally) {
  QPTaskHierarchyInput input;
  input.variables = 2;
  input.lower_bounds = Eigen::Vector2d(1.0, 0.0);
  input.upper_bounds = Eigen::Vector2d(0.0, 1.0);
  QPTaskHierarchyOutput output;
  output.solution = Eigen::Vector2d(7.0, 8.0);
  std::string error;
  EXPECT_FALSE(QPTaskHierarchy::solve(input, output, &error));
  EXPECT_EQ(error, "QP hierarchy requires at least one task level");
  EXPECT_DOUBLE_EQ(output.solution[0], 7.0);
  EXPECT_DOUBLE_EQ(output.solution[1], 8.0);
}

TEST(QPTaskHierarchy, RejectsNonFiniteTask) {
  QPTaskHierarchyInput input;
  input.variables = 1;
  QPTaskLevel level;
  level.matrix = Eigen::MatrixXd::Constant(1, 1, std::numeric_limits<double>::quiet_NaN());
  level.target = Eigen::VectorXd::Zero(1);
  input.levels.push_back(level);
  QPTaskHierarchyOutput output;
  output.solution = Eigen::VectorXd::Constant(1, 5.0);
  EXPECT_FALSE(QPTaskHierarchy::solve(input, output));
  EXPECT_DOUBLE_EQ(output.solution[0], 5.0);
}
}  // namespace
