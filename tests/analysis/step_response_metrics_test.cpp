#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mca/analysis/step_response_metrics.hpp"

namespace {

TEST(StepResponseMetrics, PositiveStepOvershootAndSettling) {
  const double time[] = {0.0, 0.1, 0.2, 0.3, 0.4};
  const double value[] = {0.0, 1.1, 1.02, 1.0, 1.0};

  mca::analysis::StepResponseMetrics metrics;
  ASSERT_TRUE(mca::analysis::compute_step_response_metrics(
                  time, value, 5U, 0.0, 1.0, 0.02, 0.1, metrics).ok());
  EXPECT_NEAR(metrics.overshoot_fraction, 0.1, 1e-12);
  EXPECT_NEAR(metrics.overshoot_percent, 10.0, 1e-12);
  EXPECT_NEAR(metrics.peak_value, 1.1, 1e-12);
  EXPECT_NEAR(metrics.peak_time_s, 0.1, 1e-12);
  EXPECT_TRUE(metrics.settled);
  EXPECT_NEAR(metrics.settling_time_s, 0.2, 1e-12);
}

TEST(StepResponseMetrics, NegativeStepUsesStepDirection) {
  const double time[] = {0.0, 0.1, 0.2, 0.3, 0.4};
  const double value[] = {1.0, -0.1, 0.02, 0.0, 0.0};

  mca::analysis::StepResponseMetrics metrics;
  ASSERT_TRUE(mca::analysis::compute_step_response_metrics(
                  time, value, 5U, 1.0, 0.0, 0.02, 0.1, metrics).ok());
  EXPECT_NEAR(metrics.overshoot_fraction, 0.1, 1e-12);
  EXPECT_NEAR(metrics.peak_value, -0.1, 1e-12);
  EXPECT_NEAR(metrics.peak_time_s, 0.1, 1e-12);
  EXPECT_TRUE(metrics.settled);
  EXPECT_NEAR(metrics.settling_time_s, 0.2, 1e-12);
}

TEST(StepResponseMetrics, NoOvershootAndExactBoundaryCanSettle) {
  const double time[] = {0.0, 0.1, 0.2, 0.3};
  const double value[] = {0.0, 0.98, 0.99, 1.0};

  mca::analysis::StepResponseMetrics metrics;
  ASSERT_TRUE(mca::analysis::compute_step_response_metrics(
                  time, value, 4U, 0.0, 1.0, 0.02, 0.1, metrics).ok());
  EXPECT_DOUBLE_EQ(metrics.overshoot_fraction, 0.0);
  EXPECT_TRUE(metrics.settled);
  EXPECT_NEAR(metrics.settling_time_s, 0.1, 1e-12);
}

TEST(StepResponseMetrics, LaterExcursionInvalidatesEarlierSettling) {
  const double time[] = {0.0, 0.1, 0.2, 0.3, 0.4};
  const double value[] = {0.0, 1.0, 1.0, 1.1, 1.0};

  mca::analysis::StepResponseMetrics metrics;
  ASSERT_TRUE(mca::analysis::compute_step_response_metrics(
                  time, value, 5U, 0.0, 1.0, 0.02, 0.1, metrics).ok());
  EXPECT_FALSE(metrics.settled);
  EXPECT_TRUE(std::isnan(metrics.settling_time_s));
}

TEST(StepResponseMetrics, NonuniformTimeSpacingIsSupported) {
  const double time[] = {0.0, 0.07, 0.13, 0.31};
  const double value[] = {0.0, 1.0, 1.0, 1.0};

  mca::analysis::StepResponseMetrics metrics;
  ASSERT_TRUE(mca::analysis::compute_step_response_metrics(
                  time, value, 4U, 0.0, 1.0, 0.02, 0.15, metrics).ok());
  EXPECT_TRUE(metrics.settled);
  // 0.07 is the earliest sample that remains in-band for the required
  // 0.15 s and remains in-band through the end of the supplied data.
  EXPECT_NEAR(metrics.settling_time_s, 0.07, 1e-12);
}

TEST(StepResponseMetrics, NeverSettles) {
  const double time[] = {0.0, 0.1, 0.2, 0.3};
  const double value[] = {0.0, 1.0, 1.1, 1.0};

  mca::analysis::StepResponseMetrics metrics;
  ASSERT_TRUE(mca::analysis::compute_step_response_metrics(
                  time, value, 4U, 0.0, 1.0, 0.02, 0.05, metrics).ok());
  EXPECT_FALSE(metrics.settled);
  EXPECT_TRUE(std::isnan(metrics.settling_time_s));
}

TEST(StepResponseMetrics, RejectsInvalidInput) {
  const double time[] = {0.0, 0.1};
  const double value[] = {0.0, 1.0};
  mca::analysis::StepResponseMetrics metrics;

  EXPECT_EQ(mca::analysis::compute_step_response_metrics(
                nullptr, value, 2U, 0.0, 1.0, 0.02, 0.05, metrics).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_EQ(mca::analysis::compute_step_response_metrics(
                time, value, 2U, 1.0, 1.0, 0.02, 0.05, metrics).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_EQ(mca::analysis::compute_step_response_metrics(
                time, value, 2U, 0.0, 1.0, 0.0, 0.05, metrics).code,
            mca::core::StatusCode::InvalidArgument);

  const double bad_time[] = {0.0, 0.1, 0.1};
  const double bad_value[] = {0.0, 1.0, 1.0};
  EXPECT_EQ(mca::analysis::compute_step_response_metrics(
                bad_time, bad_value, 3U, 0.0, 1.0, 0.02, 0.05, metrics).code,
            mca::core::StatusCode::InvalidArgument);
}

}  // namespace
