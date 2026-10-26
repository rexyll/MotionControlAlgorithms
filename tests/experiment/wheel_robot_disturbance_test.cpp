#include <limits>

#include <gtest/gtest.h>

#include "mca/simulation/step.hpp"
#include "mca/simulation/wheel_robot_disturbance.hpp"

TEST(WheelRobotDisturbanceModel, NoSlipOrDisturbancePassesCommandThrough) {
  mca::simulation::WheelRobotDisturbanceModel model;
  ASSERT_TRUE(model.configure({}));
  mca::simulation::PlanarTwistCommand actual{};
  ASSERT_TRUE(model.compute({1.2, -0.4, 0.7}, 3.0, actual));
  EXPECT_DOUBLE_EQ(actual.linear_x, 1.2);
  EXPECT_DOUBLE_EQ(actual.linear_y, -0.4);
  EXPECT_DOUBLE_EQ(actual.angular_z, 0.7);
}

TEST(WheelRobotDisturbanceModel, SlipAttenuatesEachCommandChannel) {
  mca::simulation::WheelRobotDisturbanceModel model;
  mca::simulation::WheelRobotDisturbanceConfig config;
  config.longitudinal_slip = 0.25;
  config.lateral_slip = 0.5;
  config.yaw_slip = 1.0;
  ASSERT_TRUE(model.configure(config));
  mca::simulation::PlanarTwistCommand actual{};
  ASSERT_TRUE(model.compute({2.0, 1.0, 0.8}, 0.0, actual));
  EXPECT_DOUBLE_EQ(actual.linear_x, 1.5);
  EXPECT_DOUBLE_EQ(actual.linear_y, 0.5);
  EXPECT_DOUBLE_EQ(actual.angular_z, 0.0);
}

TEST(WheelRobotDisturbanceModel, SinusoidalDisturbanceIsRepeatable) {
  mca::simulation::WheelRobotDisturbanceModel model;
  mca::simulation::WheelRobotDisturbanceConfig config;
  config.disturbance_x_amplitude = 0.4;
  config.disturbance_x_frequency = 0.5;
  config.disturbance_x_phase = 1.5707963267948966;
  config.disturbance_yaw_amplitude = 0.2;
  config.disturbance_yaw_frequency = 1.0;
  ASSERT_TRUE(model.configure(config));
  mca::simulation::PlanarTwistCommand first{};
  mca::simulation::PlanarTwistCommand second{};
  ASSERT_TRUE(model.compute({}, 0.0, first));
  ASSERT_TRUE(model.compute({}, 0.0, second));
  EXPECT_NEAR(first.linear_x, 0.4, 1e-12);
  EXPECT_DOUBLE_EQ(first.linear_x, second.linear_x);
  EXPECT_NEAR(first.angular_z, 0.0, 1e-12);
}

TEST(WheelRobotDisturbanceModel, RejectsInvalidConfigAndInputsTransactionally) {
  mca::simulation::WheelRobotDisturbanceModel model;
  mca::simulation::WheelRobotDisturbanceConfig config;
  config.lateral_slip = 1.1;
  EXPECT_FALSE(model.configure(config));
  config = {};
  config.disturbance_y_frequency = -1.0;
  EXPECT_FALSE(model.configure(config));
  config = {};
  ASSERT_TRUE(model.configure(config));
  mca::simulation::PlanarTwistCommand actual{7.0, 8.0, 9.0};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(model.compute({nan, 0.0, 0.0}, 0.0, actual));
  EXPECT_DOUBLE_EQ(actual.linear_x, 7.0);
  EXPECT_DOUBLE_EQ(actual.linear_y, 8.0);
  EXPECT_DOUBLE_EQ(actual.angular_z, 9.0);
  EXPECT_FALSE(model.compute({}, -0.1, actual));
}

TEST(WheelRobotDisturbanceModel, RejectsNonFiniteOutputWithoutMutation) {
  mca::simulation::WheelRobotDisturbanceModel model;
  mca::simulation::WheelRobotDisturbanceConfig config;
  config.disturbance_x_amplitude = std::numeric_limits<double>::max();
  config.disturbance_x_frequency = std::numeric_limits<double>::max();
  ASSERT_TRUE(model.configure(config));
  mca::simulation::PlanarTwistCommand actual{3.0, 4.0, 5.0};
  EXPECT_FALSE(model.compute({}, 2.0, actual));
  EXPECT_DOUBLE_EQ(actual.linear_x, 3.0);
  EXPECT_DOUBLE_EQ(actual.linear_y, 4.0);
  EXPECT_DOUBLE_EQ(actual.angular_z, 5.0);
}

TEST(SimulationStep, ValidatesFiniteNonNegativeTimeAndPositiveDt) {
  EXPECT_TRUE((mca::simulation::SimulationStep{0.0, 0.01, 0}).valid());
  EXPECT_TRUE((mca::simulation::SimulationStep{1.0, 0.1, 42}).valid());
  EXPECT_FALSE((mca::simulation::SimulationStep{-0.1, 0.01, 0}).valid());
  EXPECT_FALSE((mca::simulation::SimulationStep{0.0, 0.0, 0}).valid());
  EXPECT_FALSE((mca::simulation::SimulationStep{0.0, -0.1, 0}).valid());
  EXPECT_FALSE((mca::simulation::SimulationStep{std::numeric_limits<double>::infinity(), 0.1, 0}).valid());
}

TEST(SimulationClock, AdvancesDeterministicallyFromConfiguredStep) {
  mca::simulation::SimulationClock clock;
  EXPECT_FALSE(clock.advance());
  ASSERT_TRUE(clock.configure(0.01));
  ASSERT_TRUE(clock.reset());

  const auto initial = clock.current_step();
  EXPECT_DOUBLE_EQ(initial.time_s, 0.0);
  EXPECT_DOUBLE_EQ(initial.dt_s, 0.01);
  EXPECT_EQ(initial.step_index, 0U);

  ASSERT_TRUE(clock.advance());
  const auto next = clock.current_step();
  EXPECT_DOUBLE_EQ(next.time_s, 0.01);
  EXPECT_DOUBLE_EQ(next.dt_s, 0.01);
  EXPECT_EQ(next.step_index, 1U);
}

TEST(SimulationClock, InvalidConfigurationPreservesExistingClock) {
  mca::simulation::SimulationClock clock;
  ASSERT_TRUE(clock.configure(0.02));
  ASSERT_TRUE(clock.advance());

  EXPECT_FALSE(clock.configure(0.0));
  EXPECT_DOUBLE_EQ(clock.dt(), 0.02);
  EXPECT_DOUBLE_EQ(clock.time(), 0.02);
  EXPECT_EQ(clock.step_index(), 1U);

  EXPECT_FALSE(clock.configure(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_DOUBLE_EQ(clock.dt(), 0.02);
  EXPECT_DOUBLE_EQ(clock.time(), 0.02);
  EXPECT_EQ(clock.step_index(), 1U);
}

TEST(SimulationClock, ResetRestoresDeterministicBaseline) {
  mca::simulation::SimulationClock clock;
  ASSERT_TRUE(clock.configure(0.05));
  ASSERT_TRUE(clock.advance());
  ASSERT_TRUE(clock.advance());
  ASSERT_TRUE(clock.reset());

  EXPECT_DOUBLE_EQ(clock.time(), 0.0);
  EXPECT_EQ(clock.step_index(), 0U);
  EXPECT_DOUBLE_EQ(clock.current_step().dt_s, 0.05);
}

TEST(SimulationClock, OverflowingAdvancePreservesState) {
  mca::simulation::SimulationClock clock;
  ASSERT_TRUE(clock.configure(std::numeric_limits<double>::max()));
  ASSERT_TRUE(clock.advance());
  const auto before = clock.current_step();

  EXPECT_FALSE(clock.advance());
  const auto after = clock.current_step();
  EXPECT_DOUBLE_EQ(after.time_s, before.time_s);
  EXPECT_DOUBLE_EQ(after.dt_s, before.dt_s);
  EXPECT_EQ(after.step_index, before.step_index);
}
