#include <gtest/gtest.h>

#include "mca/plant/second_order.hpp"

TEST(SecondOrderPlant, RejectsInvalidInertia) {
  mca::plant::SecondOrderPlant plant;
  EXPECT_FALSE(plant.configure(0.0, 1.0).ok());
}

TEST(SecondOrderPlant, ConstantEffortReachesStaticEquilibrium) {
  mca::plant::SecondOrderPlant plant(1.0, 2.0, 10.0);
  ASSERT_TRUE(plant.reset().ok());
  for (int i = 0; i < 20000; ++i) {
    ASSERT_TRUE(plant.step(5.0, 0.0005).ok());
  }
  EXPECT_NEAR(plant.position(), 0.5, 1e-4);
  EXPECT_NEAR(plant.velocity(), 0.0, 1e-4);
}

TEST(SecondOrderPlant, FreeAccelerationMatchesInertia) {
  mca::plant::SecondOrderPlant plant(2.0, 0.0, 0.0);
  ASSERT_TRUE(plant.step(4.0, 0.1).ok());
  EXPECT_NEAR(plant.velocity(), 0.2, 1e-12);
  EXPECT_NEAR(plant.position(), 0.02, 1e-12);
}
