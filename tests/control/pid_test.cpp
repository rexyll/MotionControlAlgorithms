#include <cmath>

#include <gtest/gtest.h>

#include "mca/control/pid.hpp"
#include "mca/plant/second_order.hpp"

TEST(PidController, RejectsInvalidConfiguration) {
  mca::control::PidConfig config;
  config.output_min = 1.0;
  config.output_max = -1.0;
  mca::control::PidController pid;
  EXPECT_FALSE(pid.configure(config).ok());
  EXPECT_FALSE(pid.configured());

  config = {};
  config.integral_min = 2.0;
  config.integral_max = 1.0;
  EXPECT_FALSE(pid.configure(config).ok());
}

TEST(PidController, ProportionalResponse) {
  mca::control::PidConfig config;
  config.kp = 2.0;
  mca::control::PidController pid(config);
  ASSERT_TRUE(pid.configured());

  double output = 0.0;
  ASSERT_TRUE(pid.update(3.0, 1.0, 0.001, output).ok());
  EXPECT_DOUBLE_EQ(output, 4.0);
}

TEST(PidController, IntegralAccumulates) {
  mca::control::PidConfig config;
  config.ki = 2.0;
  mca::control::PidController pid(config);
  double output = 0.0;

  ASSERT_TRUE(pid.update(1.0, 0.0, 0.1, output).ok());
  EXPECT_NEAR(output, 0.2, 1e-12);
  ASSERT_TRUE(pid.update(1.0, 0.0, 0.1, output).ok());
  EXPECT_NEAR(output, 0.4, 1e-12);
}

TEST(PidController, OutputSaturationPreventsWindup) {
  mca::control::PidConfig config;
  config.ki = 10.0;
  config.output_min = -1.0;
  config.output_max = 1.0;
  config.integral_min = -100.0;
  config.integral_max = 100.0;
  mca::control::PidController pid(config);
  double output = 0.0;

  for (int i = 0; i < 100; ++i) {
    ASSERT_TRUE(pid.update(10.0, 0.0, 0.1, output).ok());
    EXPECT_DOUBLE_EQ(output, 1.0);
  }
  EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
}

TEST(PidController, AntiWindupRecoversWhenErrorReverses) {
  mca::control::PidConfig config;
  config.ki = 10.0;
  config.output_min = -1.0;
  config.output_max = 1.0;
  config.integral_min = -100.0;
  config.integral_max = 100.0;
  mca::control::PidController pid(config);
  double output = 0.0;

  ASSERT_TRUE(pid.reset(0.95, 0.0).ok());
  ASSERT_TRUE(pid.update(0.01, 0.0, 0.01, output).ok());
  EXPECT_DOUBLE_EQ(output, 1.0);
  EXPECT_DOUBLE_EQ(pid.integral(), 0.95);

  ASSERT_TRUE(pid.update(-0.01, 0.0, 0.01, output).ok());
  EXPECT_NEAR(pid.integral(), 0.949, 1e-12);
  EXPECT_NEAR(output, 0.849, 1e-12);
}

TEST(PidController, DerivativeUsesMeasurementAndAvoidsSetpointKick) {
  mca::control::PidConfig config;
  config.kp = 0.0;
  config.kd = 2.0;
  mca::control::PidController pid(config);
  double output = 0.0;

  ASSERT_TRUE(pid.reset(0.0, 0.0).ok());
  ASSERT_TRUE(pid.update(10.0, 0.0, 0.1, output).ok());
  EXPECT_DOUBLE_EQ(output, 0.0);

  ASSERT_TRUE(pid.update(10.0, 0.5, 0.1, output).ok());
  EXPECT_NEAR(output, -10.0, 1e-12);
}

TEST(PidController, ResetRestoresDeterministicState) {
  mca::control::PidConfig config;
  config.kp = 1.0;
  config.ki = 2.0;
  config.kd = 1.0;
  mca::control::PidController pid(config);
  double output = 0.0;

  ASSERT_TRUE(pid.update(1.0, 0.0, 0.1, output).ok());
  ASSERT_TRUE(pid.update(1.0, 0.2, 0.1, output).ok());
  ASSERT_TRUE(pid.reset().ok());
  ASSERT_TRUE(pid.update(1.0, 0.0, 0.1, output).ok());
  EXPECT_NEAR(output, 1.2, 1e-12);
}

TEST(PidController, PiTracksConstantSecondOrderPlant) {
  mca::control::PidConfig config;
  config.kp = 8.0;
  config.ki = 20.0;
  config.output_min = -20.0;
  config.output_max = 20.0;
  config.integral_min = -20.0;
  config.integral_max = 20.0;
  mca::control::PidController pid(config);
  mca::plant::SecondOrderPlant plant(1.0, 2.0, 0.0);
  ASSERT_TRUE(pid.configured());

  constexpr double dt = 0.001;
  constexpr double target = 1.0;
  double output = 0.0;
  for (int i = 0; i < 5000; ++i) {
    ASSERT_TRUE(pid.update(target, plant.position(), dt, output).ok());
    ASSERT_TRUE(plant.step(output, dt).ok());
  }

  EXPECT_NEAR(plant.position(), target, 0.01);
  EXPECT_NEAR(plant.velocity(), 0.0, 0.03);
}

TEST(PidController, PdDampsSecondOrderPlant) {
  mca::control::PidConfig config;
  config.kp = 25.0;
  config.kd = 8.0;
  config.output_min = -50.0;
  config.output_max = 50.0;
  mca::control::PidController pid(config);
  mca::plant::SecondOrderPlant plant(1.0, 0.0, 0.0);

  constexpr double dt = 0.001;
  double output = 0.0;
  for (int i = 0; i < 4000; ++i) {
    ASSERT_TRUE(pid.update(1.0, plant.position(), dt, output).ok());
    ASSERT_TRUE(plant.step(output, dt).ok());
  }

  EXPECT_NEAR(plant.position(), 1.0, 0.01);
  EXPECT_NEAR(plant.velocity(), 0.0, 0.03);
  EXPECT_TRUE(std::isfinite(output));
}
