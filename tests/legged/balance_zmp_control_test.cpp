#include <gtest/gtest.h>
#include <limits>
#include "mca/legged/balance_zmp_control.hpp"
namespace {
using namespace mca::legged;
TEST(BalanceZmpControl, StaticComProducesZmpBelowCom) {
  ZmpInput input; input.center_of_mass_world={0.03,-0.02,0.8};
  ZmpOutput output;
  ASSERT_TRUE(estimate_zmp(input,RectangularSupportRegion{-0.1,0.1,-0.1,0.1},output));
  EXPECT_NEAR(output.position_world.x,0.03,1e-12);
  EXPECT_NEAR(output.position_world.y,-0.02,1e-12);
  EXPECT_TRUE(output.inside_support);
}
TEST(BalanceZmpControl, AccelerationShiftsZmpAndDetectsInstability) {
  ZmpInput input; input.center_of_mass_world={0.0,0.0,0.8};
  input.center_of_mass_acceleration_world={2.0,0.0,0.0};
  ZmpOutput output;
  ASSERT_TRUE(estimate_zmp(input,RectangularSupportRegion{-0.1,0.1,-0.1,0.1},output));
  EXPECT_NEAR(output.position_world.x,-0.8*2.0/9.81,1e-12);
  EXPECT_FALSE(output.inside_support);
}
TEST(BalanceZmpControl, PDBalanceAccelerationIsSaturated) {
  BalanceControllerInput input;
  input.center_of_mass_position_world={1.0,0.0,0.0};
  input.maximum_acceleration={1.5,2.0,3.0};
  BalanceControllerOutput output;
  ASSERT_TRUE(compute_balance_acceleration(input,output));
  EXPECT_DOUBLE_EQ(output.desired_acceleration_world.x,-1.5);
  EXPECT_DOUBLE_EQ(output.desired_acceleration_world.y,0.0);
}
TEST(BalanceZmpControl, InvalidInputPreservesOutputs) {
  ZmpInput z; ZmpOutput zo; zo.position_world={4,5,6}; zo.inside_support=true;
  z.gravity_magnitude=0;
  EXPECT_FALSE(estimate_zmp(z,RectangularSupportRegion{},zo));
  EXPECT_DOUBLE_EQ(zo.position_world.x,4); EXPECT_TRUE(zo.inside_support);
  BalanceControllerInput b; BalanceControllerOutput bo; bo.desired_acceleration_world={7,8,9};
  b.proportional_gain.x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(compute_balance_acceleration(b,bo));
  EXPECT_DOUBLE_EQ(bo.desired_acceleration_world.x,7);
}
}
