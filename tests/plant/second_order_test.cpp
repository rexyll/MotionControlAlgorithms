#include <gtest/gtest.h>

#include <limits>

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

TEST(SecondOrderPlant, InvalidReconfigurationPreservesWorkingState) {
  mca::plant::SecondOrderPlant plant(2.0, 0.5, 3.0);
  ASSERT_TRUE(plant.reset(0.25, -0.5).ok());

  EXPECT_EQ(plant.configure(0.0, 0.0, 0.0).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_TRUE(plant.configured());
  EXPECT_DOUBLE_EQ(plant.inertia(), 2.0);
  EXPECT_DOUBLE_EQ(plant.damping(), 0.5);
  EXPECT_DOUBLE_EQ(plant.stiffness(), 3.0);
  EXPECT_DOUBLE_EQ(plant.position(), 0.25);
  EXPECT_DOUBLE_EQ(plant.velocity(), -0.5);
  ASSERT_TRUE(plant.step(1.0, 0.001).ok());
}

TEST(SecondOrderPlant, NumericalOverflowDoesNotCommitState) {
  mca::plant::SecondOrderPlant plant(1e-308, 0.0, 0.0);
  ASSERT_TRUE(plant.configured());
  ASSERT_TRUE(plant.reset(0.25, -0.5).ok());
  const double position_before = plant.position();
  const double velocity_before = plant.velocity();

  EXPECT_EQ(plant.step(std::numeric_limits<double>::max(), 1.0).code,
            mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(plant.position(), position_before);
  EXPECT_DOUBLE_EQ(plant.velocity(), velocity_before);
  EXPECT_TRUE(plant.step(0.0, 0.001).ok());
}
