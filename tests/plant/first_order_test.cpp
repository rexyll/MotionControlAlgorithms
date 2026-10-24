#include <cmath>

#include <gtest/gtest.h>

#include <limits>

#include "mca/plant/first_order.hpp"

TEST(FirstOrderPlant, RejectsInvalidTimeConstant) {
  mca::plant::FirstOrderPlant plant;
  EXPECT_FALSE(plant.configure(0.0).ok());
  EXPECT_FALSE(plant.configured());
}

TEST(FirstOrderPlant, ConvergesToConstantInput) {
  mca::plant::FirstOrderPlant plant(0.1, 2.0);
  ASSERT_TRUE(plant.configured());

  for (int i = 0; i < 3000; ++i) {
    ASSERT_TRUE(plant.step(1.5, 0.001).ok());
  }
  EXPECT_NEAR(plant.output(), 3.0, 1e-10);
}

TEST(FirstOrderPlant, ExactOneTimeConstantResponse) {
  mca::plant::FirstOrderPlant plant(0.2, 1.0);
  ASSERT_TRUE(plant.step(1.0, 0.2).ok());
  EXPECT_NEAR(plant.output(), 1.0 - std::exp(-1.0), 1e-12);
}

TEST(FirstOrderPlant, InvalidReconfigurationPreservesWorkingState) {
  mca::plant::FirstOrderPlant plant(0.1, 2.0);
  ASSERT_TRUE(plant.reset(1.25).ok());

  EXPECT_EQ(plant.configure(0.0, 9.0).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_TRUE(plant.configured());
  EXPECT_DOUBLE_EQ(plant.time_constant(), 0.1);
  EXPECT_DOUBLE_EQ(plant.gain(), 2.0);
  EXPECT_DOUBLE_EQ(plant.output(), 1.25);
  ASSERT_TRUE(plant.step(1.0, 0.01).ok());
  EXPECT_GT(plant.output(), 1.25);
}

TEST(FirstOrderPlant, NumericalOverflowDoesNotCommitOutput) {
  const double huge = std::numeric_limits<double>::max();
  mca::plant::FirstOrderPlant plant(1e-300, huge);
  ASSERT_TRUE(plant.configured());
  ASSERT_TRUE(plant.reset(0.75).ok());

  EXPECT_EQ(plant.step(huge, 1.0).code, mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(plant.output(), 0.75);
  EXPECT_TRUE(plant.step(0.0, 1.0).ok());
  EXPECT_DOUBLE_EQ(plant.output(), 0.0);
}
