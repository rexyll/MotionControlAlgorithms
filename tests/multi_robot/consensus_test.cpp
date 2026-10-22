#include "mca/multi_robot/consensus.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using mca::multi_robot::ConsensusInput;
using mca::multi_robot::ConsensusOutput;
using mca::multi_robot::consensus_step;

ConsensusInput chain_input() {
  ConsensusInput input;
  input.states.resize(3, 1);
  input.states << 0.0, 10.0, 0.0;
  input.adjacency = Eigen::MatrixXd::Zero(3, 3);
  input.adjacency(0, 1) = 1.0;
  input.adjacency(1, 0) = 1.0;
  input.adjacency(1, 2) = 1.0;
  input.adjacency(2, 1) = 1.0;
  input.gain = 1.0;
  input.time_step = 0.5;
  return input;
}

TEST(Consensus, OneStepMovesTowardNeighborAverage) {
  ConsensusInput input = chain_input();
  ConsensusOutput output;
  ASSERT_TRUE(consensus_step(input, output));
  EXPECT_DOUBLE_EQ(output.states(0, 0), 5.0);
  EXPECT_DOUBLE_EQ(output.states(1, 0), 0.0);
  EXPECT_DOUBLE_EQ(output.states(2, 0), 5.0);
}

TEST(Consensus, ConnectedUndirectedGraphConvergesToAverage) {
  ConsensusInput input = chain_input();
  ConsensusOutput output;
  for (int i = 0; i < 40; ++i) {
    ASSERT_TRUE(consensus_step(input, output));
    input.states = output.states;
  }
  EXPECT_NEAR(output.states(0, 0), 10.0 / 3.0, 1e-8);
  EXPECT_NEAR(output.states(1, 0), 10.0 / 3.0, 1e-8);
  EXPECT_NEAR(output.states(2, 0), 10.0 / 3.0, 1e-8);
}

TEST(Consensus, SupportsMultipleStateCoordinates) {
  ConsensusInput input = chain_input();
  input.states.resize(3, 2);
  input.states << 0.0, 3.0, 10.0, 6.0, 0.0, 9.0;
  ConsensusOutput output;
  ASSERT_TRUE(consensus_step(input, output));
  EXPECT_DOUBLE_EQ(output.states(0, 1), 4.5);
  EXPECT_DOUBLE_EQ(output.states(2, 1), 7.5);
}

TEST(Consensus, RejectsUnstableStepAndPreservesOutput) {
  ConsensusInput input = chain_input();
  input.time_step = 0.6;
  ConsensusOutput output;
  output.states = Eigen::MatrixXd::Constant(1, 1, 123.0);
  std::string error;
  EXPECT_FALSE(consensus_step(input, output, &error));
  EXPECT_FALSE(error.empty());
  ASSERT_EQ(output.states.rows(), 1);
  EXPECT_DOUBLE_EQ(output.states(0, 0), 123.0);
}

TEST(Consensus, RejectsInvalidInputs) {
  ConsensusInput input = chain_input();
  ConsensusOutput output;
  input.adjacency(0, 1) = -1.0;
  EXPECT_FALSE(consensus_step(input, output));
  input = chain_input();
  input.adjacency.resize(2, 2);
  EXPECT_FALSE(consensus_step(input, output));
  input = chain_input();
  input.states(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(consensus_step(input, output));
}
}  // namespace
