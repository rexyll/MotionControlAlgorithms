#include "mca/control/null_space_control.hpp"

#include <gtest/gtest.h>\n\n#include <limits>

TEST(NullSpaceControlTest, ProjectsSecondaryVelocityIntoExactNullSpace) {
  Eigen::MatrixXd jacobian(1, 3);
  jacobian << 1.0, 0.0, 0.0;
  Eigen::VectorXd secondary(3);
  secondary << 4.0, 2.0, -3.0;

  Eigen::MatrixXd projector;
  Eigen::VectorXd velocity;
  ASSERT_TRUE(mca::control::NullSpaceControl::compute(
      jacobian, secondary, projector, velocity));

  EXPECT_TRUE(projector.isApprox((Eigen::Matrix3d() <<
      0.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0).finished(), 1e-12));
  EXPECT_TRUE(velocity.isApprox((Eigen::Vector3d() << 0.0, 2.0, -3.0).finished(), 1e-12));
  EXPECT_TRUE((jacobian * velocity).isApprox(Eigen::VectorXd::Zero(1), 1e-12));
}

TEST(NullSpaceControlTest, HandlesRedundantTaskJacobian) {
  Eigen::MatrixXd jacobian(2, 3);
  jacobian << 1.0, 0.0, 0.0,
              0.0, 1.0, 0.0;
  Eigen::VectorXd secondary(3);
  secondary << 1.0, 2.0, 7.0;
  Eigen::MatrixXd projector;
  Eigen::VectorXd velocity;

  ASSERT_TRUE(mca::control::NullSpaceControl::compute(
      jacobian, secondary, projector, velocity));
  EXPECT_TRUE(velocity.isApprox((Eigen::Vector3d() << 0.0, 0.0, 7.0).finished(), 1e-12));
  EXPECT_TRUE((projector * projector).isApprox(projector, 1e-12));
}

TEST(NullSpaceControlTest, DampedProjectorSuppressesNearSingularDirection) {
  Eigen::MatrixXd jacobian(1, 2);
  jacobian << 1.0, 1e-10;
  Eigen::VectorXd secondary(2);
  secondary << 0.0, 1.0;
  Eigen::MatrixXd projector;
  Eigen::VectorXd velocity;
  ASSERT_TRUE(mca::control::NullSpaceControl::compute(
      jacobian, secondary, projector, velocity, 0.1));
  EXPECT_TRUE(projector.allFinite());
  EXPECT_TRUE(velocity.allFinite());
}

TEST(NullSpaceControlTest, InvalidInputPreservesOutputs) {
  Eigen::MatrixXd jacobian(1, 2);
  jacobian << 1.0, 0.0;
  Eigen::VectorXd secondary(1);
  secondary << 2.0;
  Eigen::MatrixXd projector = Eigen::MatrixXd::Constant(1, 1, 8.0);
  Eigen::VectorXd velocity = Eigen::VectorXd::Constant(1, 9.0);

  EXPECT_FALSE(mca::control::NullSpaceControl::compute(
      jacobian, secondary, projector, velocity));
  ASSERT_EQ(projector.rows(), 1);
  EXPECT_DOUBLE_EQ(projector(0, 0), 8.0);
  EXPECT_DOUBLE_EQ(velocity[0], 9.0);
}

TEST(NullSpaceControlTest, RejectsInvalidDamping) {
  Eigen::MatrixXd jacobian(1, 1);
  jacobian << 1.0;
  Eigen::VectorXd secondary(1);
  secondary << 0.0;
  Eigen::MatrixXd projector;
  Eigen::VectorXd velocity;
  EXPECT_FALSE(mca::control::NullSpaceControl::compute(
      jacobian, secondary, projector, velocity, -0.1));
}

TEST(NullSpaceControlTest, ExactProjectorDoesNotLeakIntoPrimaryTask) {
  Eigen::MatrixXd jacobian(2, 4);
  jacobian << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;
  Eigen::VectorXd secondary(4);
  secondary << 2.0, -1.0, 3.0, 4.0;
  Eigen::MatrixXd projector;
  Eigen::VectorXd velocity;

  ASSERT_TRUE(mca::control::NullSpaceControl::compute(
      jacobian, secondary, projector, velocity));
  EXPECT_TRUE((jacobian * projector).isApprox(
      Eigen::MatrixXd::Zero(jacobian.rows(), jacobian.cols()), 1e-12));
  EXPECT_TRUE((projector * projector).isApprox(projector, 1e-12));
}

TEST(NullSpaceControlTest, RejectsNonFiniteJacobianAndPreservesOutputs) {
  Eigen::MatrixXd jacobian(1, 2);
  jacobian << 1.0, std::numeric_limits<double>::infinity();
  Eigen::VectorXd secondary(2);
  secondary << 1.0, 2.0;
  Eigen::MatrixXd projector = Eigen::MatrixXd::Constant(2, 2, 8.0);
  Eigen::VectorXd velocity = Eigen::VectorXd::Constant(2, 9.0);

  EXPECT_FALSE(mca::control::NullSpaceControl::compute(
      jacobian, secondary, projector, velocity));
  EXPECT_TRUE(projector.isApprox(Eigen::MatrixXd::Constant(2, 2, 8.0)));
  EXPECT_TRUE(velocity.isApprox(Eigen::VectorXd::Constant(2, 9.0)));
}
