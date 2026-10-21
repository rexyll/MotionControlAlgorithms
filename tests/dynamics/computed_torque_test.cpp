#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <string>
#include <vector>
#include "mca/dynamics/computed_torque.hpp"

namespace {
using namespace mca;
using kinematics::DhModelAdapter;
using kinematics::DhParameter;
using kinematics::JointType;
using kinematics::RobotModel;
using dynamics::ComputedTorque;
using dynamics::LinkInertial;

RobotModel one_link() {
  DhParameter p; p.name = "joint"; p.type = JointType::Revolute; p.a = 0.0;
  RobotModel model; EXPECT_TRUE(DhModelAdapter::build("one", "base", {p}, model));
  return model;
}
TEST(ComputedTorqueTest, MatchesMassTimesDesiredAccelerationPlusGravity) {
  const auto model = one_link();
  std::vector<LinkInertial> inertias(2);
  inertias[1].mass = 2.0; inertias[1].center_of_mass = {0.5, 0, 0};
  Eigen::VectorXd q = Eigen::VectorXd::Zero(1), zero = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd ddq = Eigen::VectorXd::Constant(1, 2.0);
  Eigen::VectorXd kp = Eigen::VectorXd::Zero(1), kd = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd limits;
  Eigen::VectorXd output;
  ASSERT_TRUE(ComputedTorque::compute(model, inertias, q, zero, q, zero, ddq,
                                      kp, kd, {0, -9.81, 0}, limits, output));
  EXPECT_NEAR(output[0], 10.81, 1e-10); // M=0.5, ddq=2, gravity=9.81.
}
TEST(ComputedTorqueTest, AppliesPDFeedbackAndTorqueLimit) {
  const auto model = one_link();
  std::vector<LinkInertial> inertias(2);
  inertias[1].mass = 2.0; inertias[1].center_of_mass = {0.5, 0, 0};
  Eigen::VectorXd q = Eigen::VectorXd::Zero(1), dq = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd qd = Eigen::VectorXd::Constant(1, 1.0), zero = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd ddq = zero, kp = Eigen::VectorXd::Constant(1, 4.0);
  Eigen::VectorXd kd = Eigen::VectorXd::Constant(1, 2.0);
  Eigen::VectorXd limits = Eigen::VectorXd::Constant(1, 1.0), output;
  ASSERT_TRUE(ComputedTorque::compute(model, inertias, q, dq, qd, zero, ddq,
                                      kp, kd, {0, 0, 0}, limits, output));
  EXPECT_NEAR(output[0], 1.0, 1e-12);
}
TEST(ComputedTorqueTest, RejectsInvalidInputsTransactionally) {
  const auto model = one_link();
  std::vector<LinkInertial> inertias(2);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(1), zero = q;
  Eigen::VectorXd gains = Eigen::VectorXd::Ones(1), limits;
  Eigen::VectorXd output = Eigen::VectorXd::Constant(1, 42.0);
  Eigen::VectorXd bad_kp = Eigen::VectorXd::Constant(1, -1.0);
  EXPECT_FALSE(ComputedTorque::compute(model, inertias, q, zero, q, zero, zero,
                                       bad_kp, gains, {0,0,-9.81}, limits, output));
  EXPECT_DOUBLE_EQ(output[0], 42.0);
}
}  // namespace
