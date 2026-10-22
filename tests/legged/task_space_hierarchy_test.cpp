#include "mca/legged/task_space_hierarchy.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using mca::legged::TaskSpaceHierarchy;
using mca::legged::TaskSpaceHierarchyInput;
using mca::legged::TaskSpaceHierarchyOutput;
using mca::legged::TaskSpaceLevel;

TEST(TaskSpaceHierarchy, SolvesSingleTask) {
  TaskSpaceHierarchyInput input;
  input.variables = 2;
  TaskSpaceLevel task;
  task.jacobian = Eigen::MatrixXd::Identity(2, 2);
  task.target_velocity = Eigen::Vector2d(1.0, -2.0);
  input.levels.push_back(task);
  TaskSpaceHierarchyOutput output;
  ASSERT_TRUE(TaskSpaceHierarchy::solve(input, output));
  ASSERT_EQ(output.velocity.size(), 2);
  EXPECT_NEAR(output.velocity[0], 1.0, 1e-5);
  EXPECT_NEAR(output.velocity[1], -2.0, 1e-5);
}

TEST(TaskSpaceHierarchy, SecondaryTaskRespectsPrimaryNullSpace) {
  TaskSpaceHierarchyInput input;
  input.variables = 2;
  TaskSpaceLevel primary;
  primary.jacobian = Eigen::MatrixXd::Zero(1, 2);
  primary.jacobian(0, 0) = 1.0;
  primary.target_velocity = Eigen::VectorXd::Constant(1, 0.5);
  input.levels.push_back(primary);
  TaskSpaceLevel secondary;
  secondary.jacobian = Eigen::Matrix2d::Identity();
  secondary.target_velocity = Eigen::Vector2d(9.0, 2.0);
  input.levels.push_back(secondary);
  TaskSpaceHierarchyOutput output;
  ASSERT_TRUE(TaskSpaceHierarchy::solve(input, output));
  EXPECT_NEAR(output.velocity[0], 0.5, 1e-4);
  EXPECT_NEAR(output.velocity[1], 2.0, 1e-4);
}

TEST(TaskSpaceHierarchy, RejectsInvalidInputTransactionally) {
  TaskSpaceHierarchyInput input;
  input.variables = 2;
  TaskSpaceLevel task;
  task.jacobian = Eigen::MatrixXd::Identity(2, 2);
  task.target_velocity = Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
  input.levels.push_back(task);
  TaskSpaceHierarchyOutput output;
  output.velocity = Eigen::Vector2d(7.0, 8.0);
  std::string error;
  EXPECT_FALSE(TaskSpaceHierarchy::solve(input, output, &error));
  EXPECT_FALSE(error.empty());
  EXPECT_DOUBLE_EQ(output.velocity[0], 7.0);
  EXPECT_DOUBLE_EQ(output.velocity[1], 8.0);
}
}  // namespace
