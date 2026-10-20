#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mca/control/state_observer.hpp"
#include "mca/plant/second_order.hpp"

namespace {

mca::control::StateObserverConfig config() {
  mca::control::StateObserverConfig c;
  c.inertia_kg_m2 = 1.0;
  c.viscous_damping_nm_per_rad_s = 0.4;
  c.stiffness_nm_per_rad = 2.0;
  c.position_gain_per_s = 30.0;
  c.velocity_gain_per_s2 = 225.0;
  return c;
}

TEST(StateObserverTest, RejectsInvalidConfigurationAndInputs) {
  auto c = config();
  c.inertia_kg_m2 = 0.0;
  mca::control::StateObserver observer;
  EXPECT_FALSE(observer.configure(c).ok());

  c = config();
  c.position_gain_per_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(observer.configure(c).ok());

  ASSERT_TRUE(observer.configure(config()).ok());
  mca::control::StateObserverOutput output;
  EXPECT_FALSE(observer.update(0.0, 0.0, 0.0, output).ok());
  EXPECT_FALSE(observer.update(0.0, std::numeric_limits<double>::quiet_NaN(), 0.001, output).ok());
}

TEST(StateObserverTest, RejectsUpdateBeforeConfiguration) {
  mca::control::StateObserver observer;
  mca::control::StateObserverOutput output;
  EXPECT_EQ(observer.update(0.0, 0.0, 0.001, output).code,
            mca::core::StatusCode::NotConfigured);
  EXPECT_EQ(observer.reset().code, mca::core::StatusCode::NotConfigured);
}

TEST(StateObserverTest, EstimatesVelocityFromMeasuredPosition) {
  auto c = config();
  c.stiffness_nm_per_rad = 0.0;
  c.viscous_damping_nm_per_rad_s = 0.0;
  c.position_gain_per_s = 40.0;
  c.velocity_gain_per_s2 = 400.0;

  mca::control::StateObserver observer(c);
  mca::plant::SecondOrderPlant plant(1.0, 0.0, 0.0);
  ASSERT_TRUE(observer.reset(0.0, 0.0).ok());
  ASSERT_TRUE(plant.reset(0.0, 0.0).ok());

  constexpr double dt = 0.001;
  constexpr double torque = 2.0;
  mca::control::StateObserverOutput output;
  for (int i = 0; i < 4000; ++i) {
    ASSERT_TRUE(plant.step(torque, dt).ok());
    ASSERT_TRUE(observer.update(torque, plant.position(), dt, output).ok());
  }

  EXPECT_NEAR(output.position, plant.position(), 0.01);
  EXPECT_NEAR(output.velocity, plant.velocity(), 0.02);
  EXPECT_NEAR(output.velocity, 8.0, 0.02);
}

TEST(StateObserverTest, ConvergesFromLargeInitialStateError) {
  auto c = config();
  c.stiffness_nm_per_rad = 0.0;
  c.viscous_damping_nm_per_rad_s = 0.0;
  c.position_gain_per_s = 20.0;
  c.velocity_gain_per_s2 = 100.0;

  mca::control::StateObserver observer(c);
  ASSERT_TRUE(observer.reset(5.0, -3.0).ok());

  constexpr double dt = 0.001;
  constexpr double measured_position = 1.0;
  mca::control::StateObserverOutput output;
  for (int i = 0; i < 3000; ++i) {
    ASSERT_TRUE(observer.update(0.0, measured_position, dt, output).ok());
  }

  EXPECT_NEAR(output.position, measured_position, 1e-3);
  EXPECT_NEAR(output.velocity, 0.0, 1e-3);
}

TEST(StateObserverTest, ResetRestoresSpecifiedState) {
  mca::control::StateObserver observer(config());
  ASSERT_TRUE(observer.reset(1.5, -0.75).ok());
  EXPECT_DOUBLE_EQ(observer.position(), 1.5);
  EXPECT_DOUBLE_EQ(observer.velocity(), -0.75);
}

TEST(StateObserverTest, ModelTermsAffectEstimatedAcceleration) {
  auto c = config();
  c.position_gain_per_s = 0.0;
  c.velocity_gain_per_s2 = 0.0;
  mca::control::StateObserver observer(c);
  ASSERT_TRUE(observer.reset(2.0, 3.0).ok());

  mca::control::StateObserverOutput output;
  ASSERT_TRUE(observer.update(10.0, 2.0, 0.001, output).ok());
  const double expected = (10.0 - 0.4 * 3.0 - 2.0 * 2.0) / 1.0;
  EXPECT_DOUBLE_EQ(output.acceleration, expected);
}

}  // namespace
