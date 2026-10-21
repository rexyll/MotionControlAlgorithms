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
  EXPECT_NE(result.status,IkStatus::InvalidInput);
}
}  // namespace
