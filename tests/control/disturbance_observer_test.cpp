#include <limits>

#include <gtest/gtest.h>

#include "mca/control/disturbance_observer.hpp"

namespace {

mca::control::DisturbanceObserverConfig config() {
  mca::control::DisturbanceObserverConfig c;
  c.inertia_kg_m2 = 0.01;
  c.viscous_damping_nm_per_rad_s = 0.1;
  c.filter_time_constant_s = 0.01;
  c.disturbance_min_nm = -2.0;
  c.disturbance_max_nm = 2.0;
  return c;
}

TEST(DisturbanceObserver, RejectsInvalidConfigurationAndInputs) {
  mca::control::DisturbanceObserver observer;
  auto c = config();
  c.inertia_kg_m2 = 0.0;
  EXPECT_FALSE(observer.configure(c).ok());
  ASSERT_TRUE(observer.configure(config()).ok());
  mca::control::DisturbanceObserverOutput output;
  EXPECT_FALSE(observer.update(0.0, std::numeric_limits<double>::quiet_NaN(), 0.001, output).ok());
  EXPECT_FALSE(observer.update(0.0, 0.0, 0.0, output).ok());
}

TEST(DisturbanceObserver, EstimatesConstantModelResidual) {
  auto c = config();
  c.viscous_damping_nm_per_rad_s = 0.0;
  c.filter_time_constant_s = 0.01;
  mca::control::DisturbanceObserver observer(c);
  mca::control::DisturbanceObserverOutput output;
  ASSERT_TRUE(observer.reset(0.0, 0.0).ok());

  // With J=0.01 and acceleration=20 rad/s^2, J*a=0.2 Nm. Zero command
  // therefore represents a +0.2 Nm external disturbance.
  double velocity = 0.0;
  constexpr double dt = 0.001;
  for (int i = 0; i < 200; ++i) {
    velocity += 20.0 * dt;
    ASSERT_TRUE(observer.update(0.0, velocity, dt, output).ok());
  }
  EXPECT_NEAR(output.estimated_disturbance_nm, 0.2, 0.002);
}

TEST(DisturbanceObserver, CompensationOpposesKnownPositiveDisturbance) {
  auto c = config();
  c.viscous_damping_nm_per_rad_s = 0.0;
  c.filter_time_constant_s = 1.0e-6;
  mca::control::DisturbanceObserver observer(c);
  mca::control::DisturbanceObserverOutput output;
  ASSERT_TRUE(observer.reset(0.0, 0.0).ok());

  // A +0.5 Nm external disturbance gives +50 rad/s^2 acceleration for J=0.01.
  ASSERT_TRUE(observer.update(0.0, 0.05, 0.001, output).ok());
  EXPECT_NEAR(output.estimated_disturbance_nm, 0.5, 1e-9);
  ASSERT_TRUE(observer.update(1.2, 0.05, 0.001, output).ok());
  EXPECT_NEAR(output.compensated_torque_nm, 0.7, 1e-9);
}

TEST(DisturbanceObserver, AppliesEstimateLimits) {
  auto c = config();
  c.filter_time_constant_s = 1.0e-6;
  c.disturbance_min_nm = -0.5;
  c.disturbance_max_nm = 0.5;
  mca::control::DisturbanceObserver observer(c);
  mca::control::DisturbanceObserverOutput output;
  ASSERT_TRUE(observer.reset(0.0, 0.0).ok());
  ASSERT_TRUE(observer.update(-10.0, 0.0, 0.001, output).ok());
  EXPECT_DOUBLE_EQ(output.estimated_disturbance_nm, 0.5);
}

TEST(DisturbanceObserver, ResetRestoresSpecifiedState) {
  mca::control::DisturbanceObserver observer(config());
  mca::control::DisturbanceObserverOutput output;
  ASSERT_TRUE(observer.reset(-0.3, 2.0).ok());
  EXPECT_DOUBLE_EQ(observer.estimated_disturbance_nm(), -0.3);
  ASSERT_TRUE(observer.update(0.0, 2.0, 0.001, output).ok());
  EXPECT_NE(output.estimated_disturbance_nm, 0.0);
}

}  // namespace
