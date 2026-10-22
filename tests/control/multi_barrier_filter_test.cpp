#include <limits>

#include <gtest/gtest.h>

#include "mca/control/multi_barrier_filter.hpp"

namespace {

mca::control::MultiBarrierFilter make_filter() {
  mca::control::MultiBarrierFilter filter;
  mca::control::MultiBarrierFilterConfig config;
  config.alpha = 1.0;
  config.control_min = -2.0;
  config.control_max = 2.0;
  EXPECT_TRUE(filter.configure(config).ok());
  return filter;
}

TEST(MultiBarrierFilter, DetectsInfeasibleIntersectionTransactionally) {
  auto filter = make_filter();
  mca::control::MultiBarrierFilterInput input;
  input.nominal_control = -1.5;
  input.constraints = {
      {-0.5, 0.0, 1.0},
      {-0.25, 0.0, 1.0},
      {-0.5, 0.0, -1.0}
  };
  mca::control::MultiBarrierFilterOutput output;
  output.control = 77.0;
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::ConstraintViolation);
  EXPECT_DOUBLE_EQ(output.control, 77.0);
}

TEST(MultiBarrierFilter, ProjectsOntoIntersectionAndReportsActiveConstraints) {
  auto filter = make_filter();
  mca::control::MultiBarrierFilterInput input;
  input.nominal_control = -1.0;
  input.constraints = {
      {-0.4, 0.0, 1.0},
      {0.8, 0.0, -1.0},
      {1.0, -0.1, 0.0}
  };
  mca::control::MultiBarrierFilterOutput output;
  ASSERT_TRUE(filter.filter(input, output).ok());
  EXPECT_NEAR(output.control, 0.4, 1.0e-12);
  EXPECT_NEAR(output.minimum_safety_margin, 0.0, 1.0e-12);
  EXPECT_GE(output.active_constraints, 1U);
  EXPECT_TRUE(output.modified);
}

TEST(MultiBarrierFilter, LeavesSafeNominalControlUnchanged) {
  auto filter = make_filter();
  mca::control::MultiBarrierFilterInput input;
  input.nominal_control = 0.25;
  input.constraints = {{1.0, 0.0, 1.0}, {0.5, 0.0, -1.0}};
  mca::control::MultiBarrierFilterOutput output;
  ASSERT_TRUE(filter.filter(input, output).ok());
  EXPECT_DOUBLE_EQ(output.control, input.nominal_control);
  EXPECT_FALSE(output.modified);
  EXPECT_GT(output.minimum_safety_margin, 0.0);
}

TEST(MultiBarrierFilter, DetectsConstraintIndependentOfControl) {
  auto filter = make_filter();
  mca::control::MultiBarrierFilterInput input;
  input.constraints = {{-2.0, 0.0, 0.0}};
  mca::control::MultiBarrierFilterOutput output;
  output.control = 9.0;
  EXPECT_EQ(filter.filter(input, output).code,
            mca::core::StatusCode::ConstraintViolation);
  EXPECT_DOUBLE_EQ(output.control, 9.0);
}

TEST(MultiBarrierFilter, RejectsInvalidInputTransactionally) {
  auto filter = make_filter();
  mca::control::MultiBarrierFilterInput input;
  input.constraints = {{0.0, 0.0, 0.0}};
  input.nominal_control = std::numeric_limits<double>::quiet_NaN();
  mca::control::MultiBarrierFilterOutput output;
  output.control = 42.0;
  EXPECT_EQ(filter.filter(input, output).code, mca::core::StatusCode::InvalidArgument);
  EXPECT_DOUBLE_EQ(output.control, 42.0);
  input.nominal_control = 0.0;
  input.constraints.clear();
  EXPECT_EQ(filter.filter(input, output).code, mca::core::StatusCode::InvalidArgument);
  EXPECT_DOUBLE_EQ(output.control, 42.0);
}

}  // namespace
