#include <limits>

#include <gtest/gtest.h>

#include "mca/control/sliding_mode.hpp"

namespace {

mca::control::SlidingModeConfig config() {
  mca::control::SlidingModeConfig c;
  c.switching_gain = 2.0;
  c.boundary_layer = 0.1;
  c.output_min = -3.0;
  c.output_max = 3.0;
  return c;
}

TEST(SlidingModeController, SwitchingTermOpposesSlidingVariable) {
  mca::control::SlidingModeController controller(config());
  mca::control::SlidingModeOutput output;
  ASSERT_TRUE(controller.update(0.5, 1.0, output).ok());
  EXPECT_DOUBLE_EQ(output.switching_control, -2.0);
  EXPECT_DOUBLE_EQ(output.control, -1.0);
  ASSERT_TRUE(controller.update(-0.5, 1.0, output).ok());
  EXPECT_DOUBLE_EQ(output.switching_control, 2.0);
  EXPECT_DOUBLE_EQ(output.control, 3.0);
  EXPECT_TRUE(output.saturated);
}

TEST(SlidingModeController, BoundaryLayerIsContinuousAndLinearInside) {
  mca::control::SlidingModeController controller(config());
  mca::control::SlidingModeOutput output;
  ASSERT_TRUE(controller.update(0.025, 0.0, output).ok());
  EXPECT_DOUBLE_EQ(output.switching_control, -0.5);
  ASSERT_TRUE(controller.update(0.0, 0.0, output).ok());
  EXPECT_DOUBLE_EQ(output.switching_control, 0.0);
}

TEST(SlidingModeController, ClipsOutputAndReportsSaturation) {
  mca::control::SlidingModeController controller(config());
  mca::control::SlidingModeOutput output;
  ASSERT_TRUE(controller.update(0.0, 10.0, output).ok());
  EXPECT_DOUBLE_EQ(output.control, 3.0);
  EXPECT_TRUE(output.saturated);
}

TEST(SlidingModeController, RejectsInvalidConfigurationAndPreservesOutput) {
  mca::control::SlidingModeController controller;
  mca::control::SlidingModeOutput output;
  output.control = 42.0;
  EXPECT_EQ(controller.update(0.0, 0.0, output).code,
            mca::core::StatusCode::NotConfigured);
  auto c = config();
  c.boundary_layer = 0.0;
  EXPECT_FALSE(controller.configure(c).ok());
  ASSERT_TRUE(controller.configure(config()).ok());
  EXPECT_FALSE(controller.update(std::numeric_limits<double>::quiet_NaN(), 0.0, output).ok());
  EXPECT_DOUBLE_EQ(output.control, 42.0);
}

TEST(SlidingModeController, RejectsInvalidGainAndOutputBounds) {
  mca::control::SlidingModeController controller;
  auto c = config();
  c.switching_gain = -1.0;
  EXPECT_FALSE(controller.configure(c).ok());
  c = config();
  c.output_min = 4.0;
  c.output_max = 3.0;
  EXPECT_FALSE(controller.configure(c).ok());
}

}  // namespace
