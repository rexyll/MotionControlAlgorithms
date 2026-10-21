#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <string>
#include <vector>
#include "mca/dynamics/gravity_compensation.hpp"

namespace {
using namespace mca;
using kinematics::DhModelAdapter;
using kinematics::DhParameter;
using kinematics::JointType;
using kinematics::RobotModel;
using dynamics::GravityCompensation;
using dynamics::LinkInertial;

RobotModel one_link() {
  DhParameter p; p.name = "joint"; p.type = JointType::Revolute; p.a = 0.0;
  RobotModel model; EXPECT_TRUE(DhModelAdapter::build("one", "base", {p}, model));
  return model;
}
TEST(GravityCompensationTest, AppliesScaleAndPerJointLimit) {
  const auto model = one_link();
  std::vector<LinkInertial> inertias(2);
  inertias[1].mass = 2.0; inertias[1].center_of_mass = {0.5, 0, 0};
  Eigen::VectorXd output;
  Eigen::VectorXd limits(1); limits[0] = 3.0;
  ASSERT_TRUE(GravityCompensation::compute(model, inertias, Eigen::VectorXd::Zero(1),
                                            {0, -9.81, 0}, 0.5, limits, output));
  ASSERT_EQ(output.size(), 1);
  EXPECT_NEAR(output[0], 3.0, 1e-10);
}
TEST(GravityCompensationTest, EmptyLimitsMeanUnclampedAndScaleZeroDisablesFeedforward) {
  const auto model = one_link();
  std::vector<LinkInertial> inertias(2);
  inertias[1].mass = 2.0; inertias[1].center_of_mass = {0.5, 0, 0};
  Eigen::VectorXd output;
  ASSERT_TRUE(GravityCompensation::compute(model, inertias, Eigen::VectorXd::Zero(1),
                                            {0, -9.81, 0}, 0.0, Eigen::VectorXd{}, output));
  EXPECT_NEAR(output[0], 0.0, 1e-12);
  ASSERT_TRUE(GravityCompensation::compute(model, inertias, Eigen::VectorXd::Zero(1),
                                            {0, -9.81, 0}, 1.0, Eigen::VectorXd{}, output));
  EXPECT_NEAR(output[0], 9.81, 1e-10);
}
TEST(GravityCompensationTest, InvalidOptionsPreserveOutput) {
  const auto model = one_link();
  std::vector<LinkInertial> inertias(2);
  inertias[1].mass = 2.0; inertias[1].center_of_mass = {0.5, 0, 0};
  Eigen::VectorXd output = Eigen::VectorXd::Constant(1, 42.0);
  Eigen::VectorXd limits(2); limits.setOnes();
  std::string error;
  EXPECT_FALSE(GravityCompensation::compute(model, inertias, Eigen::VectorXd::Zero(1),
                                             {0, -9.81, 0}, 1.0, limits, output, &error));
  EXPECT_DOUBLE_EQ(output[0], 42.0);
  EXPECT_FALSE(GravityCompensation::compute(model, inertias, Eigen::VectorXd::Zero(1),
                                             {0, -9.81, 0}, -1.0, Eigen::VectorXd{}, output, &error));
  EXPECT_DOUBLE_EQ(output[0], 42.0);
}
}  // namespace
