#include <cmath>

#include <gtest/gtest.h>

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
