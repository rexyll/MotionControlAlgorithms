#include <gtest/gtest.h>
#include "mca/hardware/fault_injectable_virtual_joint_device.hpp"
using Device = mca::hardware::FaultInjectableVirtualJointDevice<1>;
using Category = mca::hardware::HardwareFaultCategory;
TEST(FaultInjectableVirtualJointDevice, InjectedFaultStopsAndBlocksCommands) {
  Device d; ASSERT_TRUE(d.initialize().ok()); ASSERT_TRUE(d.activate().ok());
  mca::hardware::JointEffortCommand<double,1> c; c.effort={2.0};
  ASSERT_TRUE(d.write_effort(c).ok()); ASSERT_TRUE(d.step(0.001).ok());
  ASSERT_TRUE(d.inject_fault(Category::Overcurrent,0xA501U,123U).ok());
  EXPECT_EQ(d.lifecycle_state(),mca::hardware::HardwareLifecycleState::Faulted);
  EXPECT_EQ(d.write_effort(c).code,mca::core::StatusCode::ConstraintViolation);
  mca::hardware::JointState<double,1> s; ASSERT_TRUE(d.read_state(s).ok());
  EXPECT_DOUBLE_EQ(s.effort[0],0.0);
}
TEST(FaultInjectableVirtualJointDevice, RawCodeLatchAndSafeRecovery) {
  Device d; ASSERT_TRUE(d.initialize().ok()); ASSERT_TRUE(d.activate().ok());
  ASSERT_TRUE(d.inject_fault(Category::Communication,0xDEADBEEFU,9001U).ok());
  mca::hardware::HardwareFaultSet<16> fs; ASSERT_TRUE(d.read_faults(fs).ok());
  ASSERT_EQ(fs.count,1U); EXPECT_EQ(fs.faults[0].vendor_raw_code,0xDEADBEEFU);
  EXPECT_TRUE(fs.faults[0].active); EXPECT_TRUE(fs.faults[0].latched);
  EXPECT_EQ(d.clear_faults().code,mca::core::StatusCode::ConstraintViolation);
  ASSERT_TRUE(d.resolve_fault(Category::Communication).ok());
  ASSERT_TRUE(d.read_faults(fs).ok()); EXPECT_FALSE(fs.faults[0].active);
  EXPECT_TRUE(fs.faults[0].latched);
  ASSERT_TRUE(d.clear_faults().ok());
  EXPECT_EQ(d.lifecycle_state(),mca::hardware::HardwareLifecycleState::Inactive);
  ASSERT_TRUE(d.activate().ok());
}
TEST(FaultInjectableVirtualJointDevice, MultipleCategoriesAreRetained) {
  Device d; ASSERT_TRUE(d.initialize().ok()); ASSERT_TRUE(d.activate().ok());
  ASSERT_TRUE(d.inject_fault(Category::SensorEncoder,0x11U,10U).ok());
  EXPECT_EQ(d.inject_fault(Category::SensorEncoder,0x12U,11U).code,
            mca::core::StatusCode::ConstraintViolation);
  ASSERT_TRUE(d.inject_fault(Category::Watchdog,0x22U,12U).ok());
  mca::hardware::HardwareFaultSet<16> fs; ASSERT_TRUE(d.read_faults(fs).ok());
  ASSERT_EQ(fs.count,2U); EXPECT_EQ(fs.faults[1].category,Category::Watchdog);
}
