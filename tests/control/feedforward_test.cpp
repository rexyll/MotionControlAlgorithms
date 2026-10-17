#include <limits>

#include <gtest/gtest.h>

#include "mca/control/feedforward.hpp"

TEST(PidFeedforward, ComputesVelocityAndAccelerationTerms) {
  mca::control::PidFeedforward ff({2.0, 3.0, 0.5});
  EXPECT_DOUBLE_EQ(ff.compute(1.0, 2.0), 8.5);
}

TEST(PidFeedforward, RejectsNonFiniteConfiguration) {
  mca::control::PidFeedforward ff;
  mca::control::PidFeedforwardConfig config;
  config.velocity_gain = std::numeric_limits<double>::infinity();
  EXPECT_EQ(ff.configure(config).code, mca::core::StatusCode::InvalidArgument);
}
