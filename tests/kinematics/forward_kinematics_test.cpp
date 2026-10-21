#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <gtest/gtest.h>
#include "mca/kinematics/forward_kinematics.hpp"

namespace {
using mca::geometry::Pose3D;
using mca::kinematics::DhModelAdapter;
using mca::kinematics::DhParameter;
using mca::kinematics::ForwardKinematics;
using mca::kinematics::ForwardKinematicsResult;
using mca::kinematics::JointType;
using mca::kinematics::RobotModel;

void expect_near(Pose3D a, Pose3D b, double eps = 1e-10) {
  EXPECT_NEAR(a.position.x, b.position.x, eps);
  EXPECT_NEAR(a.position.y, b.position.y, eps);
  EXPECT_NEAR(a.position.z, b.position.z, eps);
  const double dot = a.rotation.w*b.rotation.w + a.rotation.x*b.rotation.x +
                     a.rotation.y*b.rotation.y + a.rotation.z*b.rotation.z;
  EXPECT_NEAR(std::abs(dot), 1.0, eps);
}

TEST(ForwardKinematicsTest, TwoLinkPlanarDhReference) {
  std::vector<DhParameter> dh(2);
  dh[0].name = "joint1"; dh[0].type = JointType::Revolute; dh[0].a = 1.0;
  dh[1].name = "joint2"; dh[1].type = JointType::Revolute; dh[1].a = 0.5;
  RobotModel model;
  ASSERT_TRUE(DhModelAdapter::build("planar", "base", dh, model));

  ForwardKinematicsResult result;
  ASSERT_TRUE(ForwardKinematics::solve(model, {M_PI / 2.0, M_PI / 2.0}, result));
  EXPECT_NEAR(result.pose.position.x, -0.5, 1e-10);
  EXPECT_NEAR(result.pose.position.y, 1.0, 1e-10);
  EXPECT_NEAR(result.pose.position.z, 0.0, 1e-10);
  EXPECT_EQ(result.link_poses.size(), 3U);
}

TEST(ForwardKinematicsTest, FixedRevoluteAndPrismaticCoordinates) {
  RobotModel model;
  model.name = "mixed";
  model.root_link = "base";
  model.tip_link = "tip";
  model.links = {"base", "fixed", "revolute", "tip"};
  mca::kinematics::RobotJoint fixed;
  fixed.name = "fixed"; fixed.parent_link = "base"; fixed.child_link = "fixed";
  fixed.type = JointType::Fixed; fixed.origin.position = {1.0, 0.0, 0.0};
  mca::kinematics::RobotJoint revolute;
  revolute.name = "rot"; revolute.parent_link = "fixed"; revolute.child_link = "revolute";
  revolute.type = JointType::Revolute; revolute.origin.position = {1.0, 0.0, 0.0};
  mca::kinematics::RobotJoint prismatic;
  prismatic.name = "slide"; prismatic.parent_link = "revolute"; prismatic.child_link = "tip";
  prismatic.type = JointType::Prismatic; prismatic.origin.position = {0.0, 0.0, 0.0};
  prismatic.axis = {0.0, 0.0, 1.0};
  model.joints = {fixed, revolute, prismatic};

  ForwardKinematicsResult result;
  ASSERT_TRUE(ForwardKinematics::solve(model, {M_PI / 2.0, 0.25}, result));
  EXPECT_NEAR(result.pose.position.x, 1.0, 1e-10);
  EXPECT_NEAR(result.pose.position.y, 1.0, 1e-10);
  EXPECT_NEAR(result.pose.position.z, 0.25, 1e-10);
}

TEST(ForwardKinematicsTest, UrdfOriginAndAxisAreApplied) {
  const std::string urdf = R"(<robot name="r"><link name="base"/><link name="tip"/>
    <joint name="j" type="revolute"><parent link="base"/><child link="tip"/>
    <origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/><axis xyz="0 0 1"/>
    </joint></robot>)";
  RobotModel model;
  ASSERT_TRUE(mca::kinematics::UrdfModelAdapter::parse(urdf, model));
  ForwardKinematicsResult result;
  ASSERT_TRUE(ForwardKinematics::solve(model, {M_PI / 2.0}, result));
  EXPECT_NEAR(result.pose.position.x, 1.0, 1e-10);
  EXPECT_NEAR(result.pose.position.y, 0.0, 1e-10);
  const auto rotated = mca::geometry::rotate(result.pose.rotation, {1.0, 0.0, 0.0});
  EXPECT_NEAR(rotated.x, -1.0, 1e-10);
  EXPECT_NEAR(rotated.y, 0.0, 1e-10);
}

TEST(ForwardKinematicsTest, RejectsInvalidInputTransactionally) {
  std::vector<DhParameter> dh(1);
  dh[0].name = "j"; dh[0].type = JointType::Revolute; dh[0].a = 1.0;
  RobotModel model;
  ASSERT_TRUE(DhModelAdapter::build("r", "base", dh, model));
  ForwardKinematicsResult result;
  result.pose.position.x = 7.0;
  std::string error;
  EXPECT_FALSE(ForwardKinematics::solve(model, {}, result, &error));
  EXPECT_EQ(error, "joint coordinate count does not match movable joints");
  EXPECT_DOUBLE_EQ(result.pose.position.x, 7.0);

  error.clear();
  EXPECT_FALSE(ForwardKinematics::solve(model, {std::numeric_limits<double>::quiet_NaN()}, result, &error));
  EXPECT_EQ(error, "joint coordinate is non-finite");
  EXPECT_DOUBLE_EQ(result.pose.position.x, 7.0);
}

}  // namespace
