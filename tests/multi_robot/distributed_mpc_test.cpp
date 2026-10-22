#include "mca/multi_robot/distributed_mpc.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using mca::multi_robot::DistributedMpcInput;
using mca::multi_robot::DistributedMpcOutput;
using mca::multi_robot::distributed_mpc_step;

DistributedMpcInput make_input() {
  DistributedMpcInput input;
  input.control_sequences.resize(2, 4);
  input.control_sequences << 0.0, 0.2, 0.4, 0.6,
                             1.0, 0.8, 0.6, 0.4;
  input.local_gradients = Eigen::MatrixXd::Zero(2, 4);
  input.adjacency.resize(2, 2);
  input.adjacency << 0.0, 1.0, 1.0, 0.0;
  input.horizon = 2;
  input.control_dimensions = 2;
  input.gradient_step = 0.1;
  input.consensus_gain = 0.5;
  input.max_abs_control = 1.0;
  return input;
}

TEST(DistributedMpc, ConsensusPenaltyMovesPlansTowardEachOther) {
  auto input = make_input();
  DistributedMpcOutput output;
  ASSERT_TRUE(distributed_mpc_step(input, output));
  EXPECT_GT(output.control_sequences(0, 0), input.control_sequences(0, 0));
  EXPECT_LT(output.control_sequences(1, 0), input.control_sequences(1, 0));
  EXPECT_EQ(output.first_controls.rows(), 2);
  EXPECT_EQ(output.first_controls.cols(), 2);
  EXPECT_LT((output.first_controls -
             output.control_sequences.leftCols(2)).norm(), 1e-12);
}

TEST(DistributedMpc, AppliesLocalGradientAndControlBounds) {
  auto input = make_input();
  input.local_gradients = Eigen::MatrixXd::Constant(2, 4, -100.0);
  input.consensus_gain = 0.0;
  input.max_abs_control = 0.25;
  DistributedMpcOutput output;
  ASSERT_TRUE(distributed_mpc_step(input, output));
  EXPECT_LE(output.control_sequences.cwiseAbs().maxCoeff(), 0.25 + 1e-12);
}

TEST(DistributedMpc, SupportsDisconnectedGraph) {
  auto input = make_input();
  input.adjacency.setZero();
  input.local_gradients = Eigen::MatrixXd::Ones(2, 4);
  DistributedMpcOutput output;
  ASSERT_TRUE(distributed_mpc_step(input, output));
  EXPECT_NEAR(output.control_sequences(0, 0), -0.1, 1e-12);
  EXPECT_NEAR(output.control_sequences(1, 0), 0.9, 1e-12);
}

TEST(DistributedMpc, RejectsInvalidInputTransactionally) {
  auto input = make_input();
  DistributedMpcOutput output;
  output.control_sequences = Eigen::MatrixXd::Constant(1, 1, 17.0);
  output.first_controls = Eigen::MatrixXd::Constant(1, 1, 19.0);
  input.adjacency(0, 1) = -1.0;
  std::string error;
  EXPECT_FALSE(distributed_mpc_step(input, output, &error));
  EXPECT_FALSE(error.empty());
  ASSERT_EQ(output.control_sequences.rows(), 1);
  EXPECT_DOUBLE_EQ(output.control_sequences(0, 0), 17.0);
  EXPECT_DOUBLE_EQ(output.first_controls(0, 0), 19.0);
}

TEST(DistributedMpc, RejectsShapeAndNonFiniteErrors) {
  auto input = make_input();
  input.horizon = 3;
  DistributedMpcOutput output;
  EXPECT_FALSE(distributed_mpc_step(input, output));
  input = make_input();
  input.local_gradients(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(distributed_mpc_step(input, output));
  input = make_input();
  input.adjacency(0, 1) = 0.5;
  EXPECT_FALSE(distributed_mpc_step(input, output));
}
}  // namespace
