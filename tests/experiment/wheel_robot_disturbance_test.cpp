#include <limits>

#include <gtest/gtest.h>

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
