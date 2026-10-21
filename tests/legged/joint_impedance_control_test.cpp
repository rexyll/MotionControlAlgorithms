#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <limits>
#include <string>

#include "mca/legged/joint_impedance_control.hpp"

namespace {

mca::legged::JointImpedanceControlInput make_input() {
  mca::legged::JointImpedanceControlInput input;
  input.position = Eigen::Vector2d(1.0, -0.5);
  input.velocity = Eigen::Vector2d(0.2, -0.1);
  input.desired_position = Eigen::Vector2d(1.5, 0.5);
  input.desired_velocity = Eigen::Vector2d(0.0, 0.2);
  input.stiffness = Eigen::Vector2d(10.0, 20.0);
  input.damping = Eigen::Vector2d(2.0, 3.0);
  input.feedforward_torque = Eigen::Vector2d(1.0, -2.0);
  return input;
}

}  // namespace

TEST(JointImpedanceControlTest, ComputesSpringDamperAndFeedforwardTorque) {
  const auto input = make_input();
  Eigen::VectorXd output;
  ASSERT_TRUE(mca::legged::JointImpedanceControl::compute(input, output));
  EXPECT_NEAR(output[0], 5.6, 1e-12);
  EXPECT_NEAR(output[1], 18.9, 1e-12);
}

TEST(JointImpedanceControlTest, AppliesPerJointTorqueLimits) {
  auto input = make_input();
  input.torque_limits = Eigen::Vector2d(4.0, 10.0);
  Eigen::VectorXd output;
  ASSERT_TRUE(mca::legged::JointImpedanceControl::compute(input, output));
  EXPECT_NEAR(output[0], 4.0, 1e-12);
  EXPECT_NEAR(output[1], 10.0, 1e-12);
}

TEST(JointImpedanceControlTest, RejectsInvalidInputTransactionally) {
  auto input = make_input();
  Eigen::VectorXd output = Eigen::Vector2d(7.0, -8.0);
  input.damping[1] = -1.0;
  std::string error;
  EXPECT_FALSE(mca::legged::JointImpedanceControl::compute(input, output, &error));
  EXPECT_FALSE(error.empty());
  EXPECT_NEAR(output[0], 7.0, 1e-12);
  EXPECT_NEAR(output[1], -8.0, 1e-12);
}

TEST(JointImpedanceControlTest, RejectsNonFiniteInputTransactionally) {
  auto input = make_input();
  Eigen::VectorXd output = Eigen::Vector2d(3.0, 4.0);
  input.position[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mca::legged::JointImpedanceControl::compute(input, output));
  EXPECT_NEAR(output[0], 3.0, 1e-12);
  EXPECT_NEAR(output[1], 4.0, 1e-12);
}

TEST(JointImpedanceControlTest, ClosedLoopSingleJointConverges) {
  mca::legged::JointImpedanceControlInput input;
  input.position = Eigen::VectorXd::Zero(1);
  input.velocity = Eigen::VectorXd::Zero(1);
  input.desired_position = Eigen::VectorXd::Constant(1, 1.0);
  input.desired_velocity = Eigen::VectorXd::Zero(1);
  input.stiffness = Eigen::VectorXd::Constant(1, 25.0);
  input.damping = Eigen::VectorXd::Constant(1, 10.0);
  input.feedforward_torque = Eigen::VectorXd::Zero(1);

  double q = 0.0;
  double dq = 0.0;
  constexpr double dt = 0.001;
  for (int k = 0; k < 4000; ++k) {
    input.position[0] = q;
    input.velocity[0] = dq;
    Eigen::VectorXd torque;
    ASSERT_TRUE(mca::legged::JointImpedanceControl::compute(input, torque));
    const double ddq = torque[0];
    dq += dt * ddq;
    q += dt * dq;
  }
  EXPECT_NEAR(q, 1.0, 2e-3);
  EXPECT_NEAR(dq, 0.0, 2e-3);
}
