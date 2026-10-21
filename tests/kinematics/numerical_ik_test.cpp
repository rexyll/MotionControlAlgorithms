#include <cmath>
#include <limits>
#include <vector>
#include <gtest/gtest.h>
#include "mca/kinematics/numerical_ik.hpp"

namespace {
TEST(NumericalIkTest, SolvesReachableSingleRevoluteTarget) {
  using namespace mca::kinematics;
  DhParameter p; p.name="j"; p.type=JointType::Revolute; p.a=1.0;
  RobotModel model;
  ASSERT_TRUE(DhModelAdapter::build("one","base",{p},model));
  ForwardKinematicsResult target;
  ASSERT_TRUE(ForwardKinematics::solve(model,{0.7},target));
  NumericalIkOptions options; options.orientation_tolerance=1e-3;
  const auto result=NumericalIk::solve(model,target.pose,{0.1},options);
  EXPECT_EQ(result.status,IkStatus::Converged) << result.message;
  EXPECT_NEAR(result.joint_positions[0],0.7,1e-3);
}
TEST(NumericalIkTest, RejectsWrongCoordinateCountAndInvalidOptions) {
  using namespace mca::kinematics;
  DhParameter p; p.name="j"; p.type=JointType::Revolute; p.a=1.0;
  RobotModel model; ASSERT_TRUE(DhModelAdapter::build("one","base",{p},model));
  const auto bad_count=NumericalIk::solve(model,{}, {},{});
  EXPECT_EQ(bad_count.status,IkStatus::InvalidInput);
  NumericalIkOptions options; options.damping=0;
  const auto bad_options=NumericalIk::solve(model,{}, {0.0},options);
  EXPECT_EQ(bad_options.status,IkStatus::InvalidInput);
}
TEST(NumericalIkTest, ReportsIterationLimit) {
  using namespace mca::kinematics;
  DhParameter p; p.name="j"; p.type=JointType::Revolute; p.a=1.0;
  RobotModel model; ASSERT_TRUE(DhModelAdapter::build("one","base",{p},model));
  ForwardKinematicsResult target; ASSERT_TRUE(ForwardKinematics::solve(model,{1.0},target));
  NumericalIkOptions options; options.max_iterations=1;
  const auto result=NumericalIk::solve(model,target.pose,{0.0},options);
  EXPECT_EQ(result.status,IkStatus::MaxIterations);
}
TEST(NumericalIkTest, RespectsJointLimitsAndReportsStall) {
  using namespace mca::kinematics;
  DhParameter p; p.name="limited"; p.type=JointType::Revolute; p.a=1.0;
  p.limits.lower=-0.2; p.limits.upper=0.2;
  RobotModel model; ASSERT_TRUE(DhModelAdapter::build("limited","base",{p},model));
  ForwardKinematicsResult target; ASSERT_TRUE(ForwardKinematics::solve(model,{0.8},target));
  const auto result=NumericalIk::solve(model,target.pose,{0.0});
  EXPECT_EQ(result.status,IkStatus::UnreachableOrStalled);
  ASSERT_EQ(result.joint_positions.size(),1U);
  EXPECT_LE(result.joint_positions[0],0.2 + 1e-12);
  EXPECT_GE(result.joint_positions[0],-0.2 - 1e-12);
}
TEST(NumericalIkTest, RejectsNonFiniteTargetAndInvalidOptions) {
  using namespace mca::kinematics;
  DhParameter p; p.name="j"; p.type=JointType::Revolute; p.a=1.0;
  RobotModel model; ASSERT_TRUE(DhModelAdapter::build("one","base",{p},model));
  mca::geometry::Pose3D target;
  target.position.x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(NumericalIk::solve(model,target,{0.0}).status,IkStatus::InvalidInput);
  NumericalIkOptions options; options.finite_difference_step=std::numeric_limits<double>::infinity();
  EXPECT_EQ(NumericalIk::solve(model,{}, {0.0},options).status,IkStatus::InvalidInput);
}
}  // namespace
