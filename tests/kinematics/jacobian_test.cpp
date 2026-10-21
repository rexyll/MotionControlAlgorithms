#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <gtest/gtest.h>
#include "mca/kinematics/jacobian.hpp"

namespace {
using mca::kinematics::DhModelAdapter;
using mca::kinematics::DhParameter;
using mca::kinematics::Jacobian;
using mca::kinematics::JacobianResult;
using mca::kinematics::JointType;
using mca::kinematics::RobotModel;

TEST(JacobianTest, TwoLinkPlanarAnalyticalReference) {
  std::vector<DhParameter> dh(2);
  dh[0].name = "j1"; dh[0].type = JointType::Revolute; dh[0].a = 1.0;
  dh[1].name = "j2"; dh[1].type = JointType::Revolute; dh[1].a = 0.5;
  RobotModel model;
  ASSERT_TRUE(DhModelAdapter::build("planar", "base", dh, model));

  JacobianResult j;
  ASSERT_TRUE(Jacobian::solve(model, {0.0, 0.0}, j));
  ASSERT_EQ(j.columns, 2U);
  EXPECT_NEAR(j(0, 0), 0.0, 1e-12);
  EXPECT_NEAR(j(1, 0), 1.5, 1e-12);
  EXPECT_NEAR(j(5, 0), 1.0, 1e-12);
  EXPECT_NEAR(j(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(j(1, 1), 0.5, 1e-12);
  EXPECT_NEAR(j(5, 1), 1.0, 1e-12);
  EXPECT_NEAR(j(2, 0), 0.0, 1e-12);
  EXPECT_NEAR(j(3, 1), 0.0, 1e-12);
}

TEST(JacobianTest, MixedFixedRevoluteAndPrismaticChain) {
  RobotModel model;
  model.name = "mixed"; model.root_link = "base"; model.tip_link = "tip";
  model.links = {"base", "fixed", "rotated", "tip"};
  mca::kinematics::RobotJoint fixed;
  fixed.name = "fixed"; fixed.parent_link = "base"; fixed.child_link = "fixed";
  fixed.type = JointType::Fixed; fixed.origin.position = {1.0, 0.0, 0.0};
  mca::kinematics::RobotJoint revolute;
  revolute.name = "rot"; revolute.parent_link = "fixed"; revolute.child_link = "rotated";
  revolute.type = JointType::Revolute; revolute.axis = {0.0, 0.0, 1.0};
  revolute.origin.position = {1.0, 0.0, 0.0};
  mca::kinematics::RobotJoint slide;
  slide.name = "slide"; slide.parent_link = "rotated"; slide.child_link = "tip";
  slide.type = JointType::Prismatic; slide.axis = {1.0, 0.0, 0.0};
  slide.origin.rotation = mca::geometry::axis_angle({0.0, 0.0, 1.0}, 1.5707963267948966);
  model.joints = {fixed, revolute, slide};

  JacobianResult j;
  ASSERT_TRUE(Jacobian::solve(model, {0.0, 0.25}, j));
  ASSERT_EQ(j.columns, 2U);
  EXPECT_NEAR(j(5, 0), 1.0, 1e-12);
  EXPECT_NEAR(j(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(j(1, 1), 1.0, 1e-12);
  EXPECT_NEAR(j(2, 1), 0.0, 1e-12);
  EXPECT_NEAR(j(3, 1), 0.0, 1e-12);
  EXPECT_NEAR(j(4, 1), 0.0, 1e-12);
}

TEST(JacobianTest, InvalidInputLeavesOutputUnchanged) {
  DhParameter p; p.name = "j"; p.type = JointType::Revolute; p.a = 1.0;
  RobotModel model;
  ASSERT_TRUE(DhModelAdapter::build("single", "base", {p}, model));
  JacobianResult result; result.columns = 77;
  result.rows[0] = {42.0};
  std::string error;
  EXPECT_FALSE(Jacobian::solve(model, {std::numeric_limits<double>::quiet_NaN()}, result, &error));
  EXPECT_EQ(error, "joint coordinate is non-finite");
  EXPECT_EQ(result.columns, 77U);
  ASSERT_EQ(result.rows[0].size(), 1U);
  EXPECT_DOUBLE_EQ(result.rows[0][0], 42.0);
}
}  // namespace
