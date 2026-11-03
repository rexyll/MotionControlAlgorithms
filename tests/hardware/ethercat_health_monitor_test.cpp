#include <cstdint>

#include <gtest/gtest.h>

#include "mca/hardware/ethercat_health_monitor.hpp"

namespace {

using namespace mca::hardware;

EthercatBusHealthSample healthy_sample(std::uint64_t sequence) {
  EthercatBusHealthSample sample;
  sample.sequence = sequence;
  sample.diagnostics_valid = true;
  sample.link_up = true;
  sample.expected_slaves = 2U;
  sample.responding_slaves = 2U;
  sample.all_slaves_online = true;
  sample.all_slaves_operational = true;
  sample.working_counter = EthercatWorkingCounterState::Complete;
  return sample;
}

TEST(EthercatBusHealthMonitor, RequiresConsecutiveGoodSamplesAtStartup) {
  EthercatBusHealthMonitor monitor({3U, 2U});
  ASSERT_TRUE(monitor.configured());
  ASSERT_TRUE(monitor.update(healthy_sample(1U)).ok());
  EXPECT_EQ(monitor.snapshot().state, EthercatBusHealthState::Recovering);
  ASSERT_TRUE(monitor.update(healthy_sample(2U)).ok());
  EXPECT_EQ(monitor.snapshot().state, EthercatBusHealthState::Healthy);
}

TEST(EthercatBusHealthMonitor, DebouncesTransientWkcLossAndRecovers) {
  EthercatBusHealthMonitor monitor({3U, 2U});
  ASSERT_TRUE(monitor.update(healthy_sample(1U)).ok());
  ASSERT_TRUE(monitor.update(healthy_sample(2U)).ok());
  ASSERT_EQ(monitor.snapshot().state, EthercatBusHealthState::Healthy);

  auto degraded = healthy_sample(3U);
  degraded.working_counter = EthercatWorkingCounterState::Incomplete;
  ASSERT_TRUE(monitor.update(degraded).ok());
  EXPECT_EQ(monitor.snapshot().state, EthercatBusHealthState::Degraded);
  ASSERT_TRUE(monitor.update(healthy_sample(4U)).ok());
  EXPECT_EQ(monitor.snapshot().state, EthercatBusHealthState::Recovering);
  ASSERT_TRUE(monitor.update(healthy_sample(5U)).ok());
  EXPECT_EQ(monitor.snapshot().state, EthercatBusHealthState::Healthy);
}

TEST(EthercatBusHealthMonitor, EscalatesPersistentLinkOrSlaveLoss) {
  EthercatBusHealthMonitor monitor({2U, 2U});
  ASSERT_TRUE(monitor.update(healthy_sample(1U)).ok());
  ASSERT_TRUE(monitor.update(healthy_sample(2U)).ok());

  auto lost = healthy_sample(3U);
  lost.link_up = false;
  ASSERT_TRUE(monitor.update(lost).ok());
  EXPECT_EQ(monitor.snapshot().state, EthercatBusHealthState::Degraded);
  lost.sequence = 4U;
  ASSERT_TRUE(monitor.update(lost).ok());
  EXPECT_EQ(monitor.snapshot().state, EthercatBusHealthState::Lost);

  ASSERT_TRUE(monitor.update(healthy_sample(5U)).ok());
  EXPECT_EQ(monitor.snapshot().state, EthercatBusHealthState::Recovering);
  ASSERT_TRUE(monitor.update(healthy_sample(6U)).ok());
  EXPECT_EQ(monitor.snapshot().state, EthercatBusHealthState::Healthy);
}

TEST(EthercatBusHealthMonitor, TreatsDiagnosticReadFailureAsUnknownNotHealthy) {
  EthercatBusHealthMonitor monitor({2U, 2U});
  ASSERT_TRUE(monitor.update(healthy_sample(1U)).ok());
  auto invalid = healthy_sample(2U);
  invalid.diagnostics_valid = false;
  ASSERT_TRUE(monitor.update(invalid).ok());
  EXPECT_EQ(monitor.snapshot().state, EthercatBusHealthState::Unknown);
  EXPECT_FALSE(monitor.snapshot().sample_valid);
  ASSERT_TRUE(monitor.update(healthy_sample(3U)).ok());
  EXPECT_EQ(monitor.snapshot().state, EthercatBusHealthState::Recovering);
}

TEST(EthercatBusHealthMonitor, LatchesExplicitFaultUntilSupervisoryClear) {
  EthercatBusHealthMonitor monitor({2U, 2U});
  auto fault = healthy_sample(1U);
  fault.latched_fault = true;
  ASSERT_TRUE(monitor.update(fault).ok());
  EXPECT_EQ(monitor.snapshot().state, EthercatBusHealthState::Fault);
  EXPECT_EQ(monitor.clear_latched_fault().code, mca::core::StatusCode::Ok);
  ASSERT_TRUE(monitor.update(healthy_sample(2U)).ok());
  EXPECT_EQ(monitor.snapshot().state, EthercatBusHealthState::Recovering);
}

TEST(EthercatBusHealthMonitor, RejectsInvalidConfigAndRepeatedSequences) {
  EthercatBusHealthMonitor invalid({0U, 2U});
  EXPECT_FALSE(invalid.configured());
  EXPECT_EQ(invalid.update(healthy_sample(1U)).code,
            mca::core::StatusCode::InvalidArgument);

  EthercatBusHealthMonitor monitor;
  EXPECT_EQ(monitor.update(healthy_sample(1U)).code, mca::core::StatusCode::Ok);
  EXPECT_EQ(monitor.update(healthy_sample(1U)).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_EQ(monitor.update(healthy_sample(0U)).code,
            mca::core::StatusCode::InvalidArgument);
}

}  // namespace
