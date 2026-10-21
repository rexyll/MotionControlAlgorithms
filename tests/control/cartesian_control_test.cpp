#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <vector>
#include "mca/control/cartesian_control.hpp"
#include "mca/kinematics/dh_urdf_model.hpp"

namespace {
using namespace mca;
using kinematics::DhModelAdapter;
using kinematics::DhParameter;
using kinematics::JointType;
using kinematics::RobotModel;
using control::CartesianControl;

RobotModel one_link() {
  DhParameter p; p.name = "joint"; p.type = JointType::Revolute; p.a = 1.0;
  RobotModel model; EXPECT_TRUE(DhModelAdapter::build("one", "base", {p}, model));
  return model;
}

TEST(CartesianControlTest, MapsPositionErrorThroughJacobianTranspose) {
  const auto model = one_link();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(1), dq = Eigen::VectorXd::Zero(1);
  geometry::Pose3D desired;
  desired.position = {1.0, 0.1, 0.0};
  Eigen::VectorXd twist = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd kp = Eigen::VectorXd::Zero(6), kd = Eigen::VectorXd::Zero(6);
  kp[1] = 10.0;
  Eigen::VectorXd limits, output;
  ASSERT_TRUE(CartesianControl::compute(model, q, dq, desired, twist, wrench,
                                        kp, kd, limits, output));
  EXPECT_NEAR(output[0], 1.0, 1e-12);
}

TEST(CartesianControlTest, TracksOrientationAndAppliesTorqueLimit) {
  const auto model = one_link();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(1), dq = Eigen::VectorXd::Zero(1);
  geometry::Pose3D desired;
  desired.rotation = geometry::axis_angle({0, 0, 1}, 0.2);
  Eigen::VectorXd twist = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd kp = Eigen::VectorXd::Zero(6), kd = Eigen::VectorXd::Zero(6);
  kp[5] = 20.0;
  Eigen::VectorXd limits = Eigen::VectorXd::Constant(1, 0.5), output;
  ASSERT_TRUE(CartesianControl::compute(model, q, dq, desired, twist, wrench,
                                        kp, kd, limits, output));
  EXPECT_NEAR(output[0], 0.5, 1e-12);
}

TEST(CartesianControlTest, RejectsInvalidGainsTransactionally) {
  const auto model = one_link();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(1), dq = q;
  geometry::Pose3D desired;
  Eigen::VectorXd twist = Eigen::VectorXd::Zero(6), wrench = twist;
  Eigen::VectorXd kp = Eigen::VectorXd::Zero(6), kd = Eigen::VectorXd::Zero(6);
  kp[0] = -1.0;
  Eigen::VectorXd limits, output = Eigen::VectorXd::Constant(1, 7.0);
  EXPECT_FALSE(CartesianControl::compute(model, q, dq, desired, twist, wrench,
                                         kp, kd, limits, output));
  EXPECT_DOUBLE_EQ(output[0], 7.0);
}
}  // namespace
