#include <cmath>
#include <limits>
#include <string>
#include <gtest/gtest.h>
#include "mca/kinematics/singularity_metrics.hpp"

namespace {
using mca::kinematics::JacobianResult;
using mca::kinematics::SingularityMetrics;
using mca::kinematics::SingularityMetricsOptions;
using mca::kinematics::SingularityMetricsResult;

TEST(SingularityMetricsTest, FullRankDiagonalJacobianReportsConditionAndManipulability) {
  JacobianResult j; j.columns = 2U;
  for (auto& row : j.rows) row.assign(2U, 0.0);
  j.rows[0][0] = 3.0;
  j.rows[1][1] = 2.0;
  SingularityMetricsResult result;
  ASSERT_TRUE(SingularityMetrics::solve(j, result));
  ASSERT_EQ(result.singular_values.size(), 2U);
  EXPECT_NEAR(result.singular_values[0], 3.0, 1e-10);
  EXPECT_NEAR(result.singular_values[1], 2.0, 1e-10);
  EXPECT_EQ(result.numerical_rank, 2U);
  EXPECT_FALSE(result.singular);
  EXPECT_NEAR(result.condition_number, 1.5, 1e-10);
  EXPECT_NEAR(result.manipulability, 6.0, 1e-10);
}

TEST(SingularityMetricsTest, RankDeficiencyIsReportedWithInfiniteConditionNumber) {
  JacobianResult j; j.columns = 2U;
  for (auto& row : j.rows) row.assign(2U, 0.0);
  j.rows[0][0] = 1.0;
  j.rows[0][1] = 1.0;
  SingularityMetricsResult result;
  ASSERT_TRUE(SingularityMetrics::solve(j, result));
  EXPECT_EQ(result.numerical_rank, 1U);
  EXPECT_TRUE(result.singular);
  EXPECT_TRUE(std::isinf(result.condition_number));
  EXPECT_NEAR(result.minimum_singular_value, 0.0, 1e-10);
}

TEST(SingularityMetricsTest, HandlesSixByManyJacobian) {
  JacobianResult j; j.columns = 7U;
  for (auto& row : j.rows) row.assign(7U, 0.0);
  for (std::size_t i = 0; i < 6U; ++i) j.rows[i][i] = static_cast<double>(i + 1U);
  SingularityMetricsResult result;
  ASSERT_TRUE(SingularityMetrics::solve(j, result));
  EXPECT_EQ(result.singular_values.size(), 6U);
  EXPECT_EQ(result.numerical_rank, 6U);
  EXPECT_NEAR(result.maximum_singular_value, 6.0, 1e-10);
  EXPECT_NEAR(result.minimum_singular_value, 1.0, 1e-10);
}

TEST(SingularityMetricsTest, RejectsMalformedInputTransactionally) {
  JacobianResult j; j.columns = 2U;
  for (auto& row : j.rows) row.assign(2U, 0.0);
  j.rows[3].pop_back();
  SingularityMetricsResult result; result.numerical_rank = 42U;
  std::string error;
  EXPECT_FALSE(SingularityMetrics::solve(j, result, {}, &error));
  EXPECT_EQ(error, "Jacobian row size mismatch");
  EXPECT_EQ(result.numerical_rank, 42U);
  for (auto& row : j.rows) row.assign(2U, 0.0);
  SingularityMetricsOptions bad; bad.relative_rank_tolerance = -1.0;
  EXPECT_FALSE(SingularityMetrics::solve(j, result, bad, &error));
  EXPECT_EQ(error, "invalid singularity metric options");
}
}  // namespace
