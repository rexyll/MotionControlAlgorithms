#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include "mca/dynamics/rigid_body_dynamics.hpp"

namespace {
using namespace mca;
using kinematics::DhModelAdapter;
using kinematics::DhParameter;
using kinematics::JointType;
using kinematics::RobotModel;
using dynamics::LinkInertial;
using dynamics::RigidBodyDynamics;

RobotModel one_link() {
  DhParameter p; p.name = "joint"; p.type = JointType::Revolute; p.a = 1.0;
  RobotModel model; EXPECT_TRUE(DhModelAdapter::build("one", "base", {p}, model));
  return model;
}
LinkInertial point_mass(double mass, double x) {
  LinkInertial p; p.mass = mass; p.center_of_mass = {x, 0, 0}; return p;
}

TEST(RigidBodyDynamicsTest, PlanarPointMassMassMatrixReference) {
  const auto model = one_link();
  std::vector<LinkInertial> inertias(2);
  inertias[1] = point_mass(2.0, 0.5);
  Eigen::MatrixXd m;
  ASSERT_TRUE(RigidBodyDynamics::mass_matrix(model, inertias, Eigen::VectorXd::Zero(1), m));
  ASSERT_EQ(m.rows(), 1);
  EXPECT_NEAR(m(0,0), 0.5, 1e-10);  // m*r^2, no rotational inertia.
}
TEST(RigidBodyDynamicsTest, GravityMatchesPotentialGradient) {
  const auto model = one_link();
  std::vector<LinkInertial> inertias(2);
  inertias[1] = point_mass(2.0, 0.5);
  Eigen::VectorXd g;
  ASSERT_TRUE(RigidBodyDynamics::gravity(model, inertias, Eigen::VectorXd::Zero(1),
                                         {0, -9.81, 0}, g));
  ASSERT_EQ(g.size(), 1);
  EXPECT_NEAR(g[0], 9.81, 1e-10);
}
TEST(RigidBodyDynamicsTest, ZeroVelocityHasZeroCoriolis) {
  const auto model = one_link();
  std::vector<LinkInertial> inertias(2);
  inertias[1] = point_mass(2.0, 0.5);
  Eigen::VectorXd c;
  ASSERT_TRUE(RigidBodyDynamics::coriolis(model, inertias, Eigen::VectorXd::Constant(1,0.3),
                                          Eigen::VectorXd::Zero(1), c));
  EXPECT_NEAR(c[0], 0.0, 1e-12);
}
TEST(RigidBodyDynamicsTest, RejectsInvalidInputsTransactionally) {
  const auto model = one_link();
  std::vector<LinkInertial> inertias(2);
  inertias[1] = point_mass(2.0, 0.5);
  Eigen::MatrixXd m = Eigen::MatrixXd::Constant(1,1,77.0);
  std::string error;
  EXPECT_FALSE(RigidBodyDynamics::mass_matrix(model, inertias, Eigen::VectorXd::Zero(0), m, &error));
  EXPECT_EQ(error, "joint position vector is invalid");
  EXPECT_DOUBLE_EQ(m(0,0),77.0);
  inertias[1].mass = -1.0;
  EXPECT_FALSE(RigidBodyDynamics::mass_matrix(model, inertias, Eigen::VectorXd::Zero(1), m, &error));
  EXPECT_EQ(error, "link inertial parameters are invalid");
  EXPECT_DOUBLE_EQ(m(0,0),77.0);
}
TEST(RigidBodyDynamicsTest, InertiaTensorAddsRotationalMass) {
  const auto model = one_link();
  std::vector<LinkInertial> inertias(2);
  inertias[1] = point_mass(2.0, 0.5);
  inertias[1].inertia_at_com(2,2) = 0.3;
  Eigen::MatrixXd m;
  ASSERT_TRUE(RigidBodyDynamics::mass_matrix(model, inertias, Eigen::VectorXd::Zero(1), m));
  EXPECT_NEAR(m(0,0), 0.8, 1e-10);
}
}  // namespace
