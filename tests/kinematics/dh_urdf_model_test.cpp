#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "mca/kinematics/dh_urdf_model.hpp"

namespace {

TEST(DhModelAdapterTest, BuildsStandardDhSerialChain) {
  using namespace mca::kinematics;
  std::vector<DhParameter> dh{
      {"joint1", JointType::Revolute, 0.4, 0.0, 0.2, 0.1, {-1.5, 1.5, 10.0, 2.0}},
      {"joint2", JointType::Prismatic, 0.0, 1.5707963267948966, 0.3, 0.0, {0.0, 0.5, 5.0, 1.0}}};
  RobotModel model;
  std::string error;
  ASSERT_TRUE(DhModelAdapter::build("two_link", "base", dh, model, &error)) << error;
  EXPECT_TRUE(model.valid(&error)) << error;
  ASSERT_EQ(model.links.size(), 3U);
  ASSERT_EQ(model.joints.size(), 2U);
  EXPECT_EQ(model.joints[0].parent_link, "base");
  EXPECT_EQ(model.joints[1].parent_link, "link_1");
  EXPECT_EQ(model.tip_link, "link_2");
  EXPECT_NEAR(model.joints[0].origin.position.x, 0.4 * std::cos(0.1), 1e-12);
  EXPECT_NEAR(model.joints[0].origin.position.y, 0.4 * std::sin(0.1), 1e-12);
}

TEST(DhModelAdapterTest, RejectsInvalidParametersTransactionally) {
  using namespace mca::kinematics;
  RobotModel original;
  std::vector<DhParameter> valid{{"j1", JointType::Revolute, 1.0, 0.0, 0.0, 0.0, {}}};
  ASSERT_TRUE(DhModelAdapter::build("one", "base", valid, original));
  const auto before = original;
  std::vector<DhParameter> invalid{{"j1", JointType::Revolute, 1.0, 0.0, 0.0, 0.0, {1.0, -1.0, 1.0, 1.0}}};
  std::string error;
  EXPECT_FALSE(DhModelAdapter::build("bad", "base", invalid, original, &error));
  EXPECT_EQ(original.name, before.name);
  EXPECT_EQ(original.links, before.links);
  EXPECT_EQ(original.joints.size(), before.joints.size());
}

TEST(UrdfModelAdapterTest, ParsesMultilineSerialChainAndOrdersJoints) {
  using namespace mca::kinematics;
  const std::string urdf = R"(<robot name="demo">
  <link name="base"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="tool"/>
  <joint name="j2" type="prismatic">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.2" rpy="0 0 1.5707963267948966"/>
    <axis xyz="0 2 0"/>
    <limit lower="0" upper="0.4" effort="10" velocity="2"/>
  </joint>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="link1"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 2"/>
    <limit lower="-1" upper="1" effort="20" velocity="3"/>
  </joint>
  <joint name="tool_fixed" type="fixed">
    <parent link="link2"/><child link="tool"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
</robot>)";
  RobotModel model;
  std::string error;
  ASSERT_TRUE(UrdfModelAdapter::parse(urdf, model, &error)) << error;
  ASSERT_TRUE(model.valid(&error)) << error;
  ASSERT_EQ(model.root_link, "base");
  ASSERT_EQ(model.tip_link, "tool");
  ASSERT_EQ(model.joints.size(), 3U);
  EXPECT_EQ(model.joints[0].name, "j1");
  EXPECT_EQ(model.joints[1].name, "j2");
  EXPECT_EQ(model.joints[2].name, "tool_fixed");
  EXPECT_EQ(model.joints[0].type, JointType::Revolute);
  EXPECT_EQ(model.joints[1].type, JointType::Prismatic);
  EXPECT_EQ(model.joints[2].type, JointType::Fixed);
  EXPECT_NEAR(model.joints[0].axis.z, 1.0, 1e-12);
  EXPECT_NEAR(model.joints[1].axis.y, 1.0, 1e-12);
  EXPECT_NEAR(model.joints[1].limits.upper, 0.4, 1e-12);
  EXPECT_NEAR(model.joints[1].origin.position.z, 0.2, 1e-12);
}

TEST(UrdfModelAdapterTest, SupportsContinuousJointAndDefaultAxis) {
  using namespace mca::kinematics;
  const std::string urdf = R"(<robot name="continuous">
<link name="base"/><link name="tip"/>
<joint name="spin" type="continuous"><parent link="base"/><child link="tip"/></joint>
</robot>)";
  RobotModel model;
  std::string error;
  ASSERT_TRUE(UrdfModelAdapter::parse(urdf, model, &error)) << error;
  ASSERT_EQ(model.joints.size(), 1U);
  EXPECT_EQ(model.joints[0].type, JointType::Continuous);
  EXPECT_NEAR(model.joints[0].axis.z, 1.0, 1e-12);
  EXPECT_TRUE(std::isinf(model.joints[0].limits.lower));
}

TEST(UrdfModelAdapterTest, RejectsBranchAndMalformedJoint) {
  using namespace mca::kinematics;
  const std::string branch = R"(<robot name="branch"><link name="base"/><link name="a"/><link name="b"/><link name="c"/>
<joint name="ja" type="fixed"><parent link="base"/><child link="a"/></joint>
<joint name="jb" type="fixed"><parent link="base"/><child link="b"/></joint>
<joint name="jc" type="fixed"><parent link="a"/><child link="c"/></joint></robot>)";
  RobotModel model;
  std::string error;
  EXPECT_FALSE(UrdfModelAdapter::parse(branch, model, &error));
  EXPECT_FALSE(error.empty());

  const std::string malformed = R"(<robot name="bad"><link name="base"/><link name="tip"/>
<joint name="j" type="unsupported"><parent link="base"/><child link="tip"/></joint></robot>)";
  error.clear();
  EXPECT_FALSE(UrdfModelAdapter::parse(malformed, model, &error));
  EXPECT_FALSE(error.empty());
}

}  // namespace
