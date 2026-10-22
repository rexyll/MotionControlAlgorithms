#include <limits>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "mca/control/iterative_learning.hpp"

namespace {

mca::control::IterativeLearningConfig config() {
  mca::control::IterativeLearningConfig c;
  c.learning_gain = 0.5;
  c.retention_factor = 0.8;
  c.control_min = -2.0;
  c.control_max = 2.0;
  return c;
}

TEST(IterativeLearningController, LearnsErrorAcrossRepeatedTrials) {
  mca::control::IterativeLearningController controller;
  ASSERT_TRUE(controller.configure(config(), 3).ok());
  Eigen::VectorXd error(3);
  error << 1.0, -0.5, 0.25;
  Eigen::VectorXd next;
  ASSERT_TRUE(controller.update(error, next).ok());
  Eigen::VectorXd expected(3);
  expected << 0.5, -0.25, 0.125;
  EXPECT_TRUE(next.isApprox(expected, 1e-12));
  ASSERT_TRUE(controller.update(error, next).ok());
  EXPECT_TRUE(next.isApprox(0.8 * expected + 0.5 * error, 1e-12));
}

TEST(IterativeLearningController, ResetProjectsAndUpdateSaturates) {
  auto c = config();
  mca::control::IterativeLearningController controller;
  ASSERT_TRUE(controller.configure(c, 2).ok());
  Eigen::VectorXd initial(2);
  initial << 10.0, -10.0;
  ASSERT_TRUE(controller.reset(initial).ok());
  EXPECT_DOUBLE_EQ(controller.feedforward_sequence()[0], 2.0);
  EXPECT_DOUBLE_EQ(controller.feedforward_sequence()[1], -2.0);
  Eigen::VectorXd error = Eigen::VectorXd::Constant(2, 100.0);
  Eigen::VectorXd next;
  ASSERT_TRUE(controller.update(error, next).ok());
  EXPECT_DOUBLE_EQ(next[0], 2.0);
  EXPECT_DOUBLE_EQ(next[1], 2.0);
}

TEST(IterativeLearningController, RejectsInvalidInputWithoutChangingStateOrOutput) {
  mca::control::IterativeLearningController controller;
  Eigen::VectorXd error = Eigen::VectorXd::Ones(2);
  Eigen::VectorXd next = Eigen::VectorXd::Constant(1, 42.0);
  EXPECT_EQ(controller.update(error, next).code, mca::core::StatusCode::NotConfigured);
  ASSERT_TRUE(controller.configure(config(), 2).ok());
  const Eigen::VectorXd before = controller.feedforward_sequence();
  error[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(controller.update(error, next).ok());
  EXPECT_TRUE(controller.feedforward_sequence().isApprox(before));
  ASSERT_EQ(next.size(), 1);
  EXPECT_DOUBLE_EQ(next[0], 42.0);
}

TEST(IterativeLearningController, ValidatesConfigurationAndDimensions) {
  mca::control::IterativeLearningController controller;
  auto c = config();
  c.retention_factor = 1.5;
  EXPECT_FALSE(controller.configure(c, 2).ok());
  c = config();
  EXPECT_FALSE(controller.configure(c, 0).ok());
  ASSERT_TRUE(controller.configure(config(), 2).ok());
  Eigen::VectorXd wrong_size = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd next;
  EXPECT_FALSE(controller.update(wrong_size, next).ok());
}

}  // namespace
