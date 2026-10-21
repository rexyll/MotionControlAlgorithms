#include <Eigen/Dense>
#include <gtest/gtest.h>
#include "mca/control/cartesian_impedance.hpp"
#include "mca/kinematics/dh_urdf_model.hpp"

namespace {
using namespace mca;
using kinematics::DhModelAdapter;
using kinematics::DhParameter;
using kinematics::JointType;
using kinematics::RobotModel;
using control::CartesianImpedance;

RobotModel one_link() {
  DhParameter p; p.name = "joint"; p.type = JointType::Revolute; p.a = 1.0;
  RobotModel model; EXPECT_TRUE(DhModelAdapter::build("one", "base", {p}, model));
  return model;
}

TEST(CartesianImpedanceTest, VirtualSpringMapsForceThroughJacobianTranspose) {
  const auto model = one_link();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(1), dq = Eigen::VectorXd::Zero(1);
  geometry::Pose3D equilibrium;
  equilibrium.position = {1.0, 0.1, 0.0};
  const auto zero6 = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd stiffness = Eigen::VectorXd::Zero(6), damping = Eigen::VectorXd::Zero(6);
  stiffness[1] = 10.0;
  Eigen::VectorXd output;
  ASSERT_TRUE(CartesianImpedance::compute(model, q, dq, equilibrium, zero6, zero6,
                                          stiffness, damping, Eigen::VectorXd(), output));
  EXPECT_NEAR(output[0], 1.0, 1e-12);
}

TEST(CartesianImpedanceTest, DampingOpposesCartesianVelocity) {
  const auto model = one_link();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd dq = Eigen::VectorXd::Constant(1, 0.2);
  geometry::Pose3D equilibrium;
  const auto zero6 = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd damping = Eigen::VectorXd::Zero(6), stiffness = Eigen::VectorXd::Zero(6);
  damping[1] = 5.0;
  Eigen::VectorXd output;
  ASSERT_TRUE(CartesianImpedance::compute(model, q, dq, equilibrium, zero6, zero6,
                                          stiffness, damping, Eigen::VectorXd(), output));
  EXPECT_NEAR(output[0], -1.0, 1e-12);
}

TEST(CartesianImpedanceTest, WrenchLimitProtectsVirtualSpringOutput) {
  const auto model = one_link();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(1), dq = Eigen::VectorXd::Zero(1);
  geometry::Pose3D equilibrium;
  equilibrium.position = {1.0, 0.5, 0.0};
  const auto zero6 = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd stiffness = Eigen::VectorXd::Zero(6), damping = Eigen::VectorXd::Zero(6);
  stiffness[1] = 20.0;
  Eigen::VectorXd limits = Eigen::VectorXd::Constant(6, 0.25), output;
  ASSERT_TRUE(CartesianImpedance::compute(model, q, dq, equilibrium, zero6, zero6,
                                          stiffness, damping, limits, output));
  EXPECT_NEAR(output[0], 0.25, 1e-12);
}

TEST(CartesianImpedanceTest, RejectsInvalidStiffnessTransactionally) {
  const auto model = one_link();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(1), dq = q;
  geometry::Pose3D equilibrium;
  const auto zero6 = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd stiffness = Eigen::VectorXd::Zero(6), damping = Eigen::VectorXd::Zero(6);
  stiffness[0] = -1.0;
  Eigen::VectorXd output = Eigen::VectorXd::Constant(1, 7.0);
  EXPECT_FALSE(CartesianImpedance::compute(model, q, dq, equilibrium, zero6, zero6,
                                           stiffness, damping, Eigen::VectorXd(), output));
  EXPECT_DOUBLE_EQ(output[0], 7.0);
}
}  // namespace
