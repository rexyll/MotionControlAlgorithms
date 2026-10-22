#include "mca/multi_robot/communication_delay_loss.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using mca::multi_robot::CommunicationDelayLossInput;
using mca::multi_robot::CommunicationDelayLossOutput;
using mca::multi_robot::communication_delay_loss_step;

CommunicationDelayLossInput make_input() {
  CommunicationDelayLossInput input;
  input.current_messages.resize(3, 2);
  input.current_messages << 1.0, 10.0, 3.0, 30.0, 5.0, 50.0;
  input.previous_messages.resize(3, 2);
  input.previous_messages << 0.0, 0.0, 2.0, 20.0, 4.0, 40.0;
  input.last_received_aggregates = Eigen::MatrixXd::Constant(3, 2, -7.0);
  input.adjacency = Eigen::MatrixXd::Zero(3, 3);
  input.adjacency(0, 1) = 1.0;
  input.adjacency(0, 2) = 3.0;
  input.adjacency(1, 0) = 1.0;
  input.adjacency(2, 0) = 1.0;
  input.delay_steps = Eigen::MatrixXd::Zero(3, 3);
  input.packet_delivered = Eigen::MatrixXd::Ones(3, 3);
  input.packet_delivered.diagonal().setZero();
  return input;
}

TEST(CommunicationDelayLoss, UsesPerLinkCurrentOrOneStepDelayedMessage) {
  auto input = make_input();
  input.delay_steps(0, 1) = 1.0;
  CommunicationDelayLossOutput output;
  ASSERT_TRUE(communication_delay_loss_step(input, output));
  // Receiver 0 combines sender 1's previous message (weight 1) and sender 2's current (weight 3).
  EXPECT_NEAR(output.received_aggregates(0, 0), (2.0 + 3.0 * 5.0) / 4.0, 1e-12);
  EXPECT_NEAR(output.received_aggregates(0, 1), (20.0 + 3.0 * 50.0) / 4.0, 1e-12);
  EXPECT_EQ(output.fresh_message_count(0), 2);
  EXPECT_EQ(output.held_last_value(0), 0);
}

TEST(CommunicationDelayLoss, DropsPacketsAndRenormalizesRemainingWeights) {
  auto input = make_input();
  input.packet_delivered(0, 2) = 0.0;
  CommunicationDelayLossOutput output;
  ASSERT_TRUE(communication_delay_loss_step(input, output));
  EXPECT_DOUBLE_EQ(output.received_aggregates(0, 0), 3.0);
  EXPECT_DOUBLE_EQ(output.received_aggregates(0, 1), 30.0);
  EXPECT_EQ(output.fresh_message_count(0), 1);
  EXPECT_EQ(output.held_last_value(0), 0);
}

TEST(CommunicationDelayLoss, HoldsLastAggregateWhenAllIncomingPacketsDrop) {
  auto input = make_input();
  input.packet_delivered.row(0).setZero();
  CommunicationDelayLossOutput output;
  ASSERT_TRUE(communication_delay_loss_step(input, output));
  EXPECT_DOUBLE_EQ(output.received_aggregates(0, 0), -7.0);
  EXPECT_DOUBLE_EQ(output.received_aggregates(0, 1), -7.0);
  EXPECT_EQ(output.fresh_message_count(0), 0);
  EXPECT_EQ(output.held_last_value(0), 1);
}

TEST(CommunicationDelayLoss, RejectsInvalidMasksTopologyAndNonFiniteInputTransactionally) {
  auto input = make_input();
  CommunicationDelayLossOutput output;
  output.received_aggregates = Eigen::MatrixXd::Constant(1, 1, 42.0);
  output.fresh_message_count = Eigen::VectorXi::Constant(1, 7);
  output.held_last_value = Eigen::VectorXi::Constant(1, 9);
  std::string error;
  input.delay_steps(0, 1) = 2.0;
  EXPECT_FALSE(communication_delay_loss_step(input, output, &error));
  EXPECT_FALSE(error.empty());
  ASSERT_EQ(output.received_aggregates.rows(), 1);
  EXPECT_DOUBLE_EQ(output.received_aggregates(0, 0), 42.0);
  EXPECT_EQ(output.fresh_message_count(0), 7);
  input = make_input();
  input.packet_delivered(0, 1) = 0.5;
  EXPECT_FALSE(communication_delay_loss_step(input, output));
  input = make_input();
  input.adjacency(0, 0) = 1.0;
  EXPECT_FALSE(communication_delay_loss_step(input, output));
  input = make_input();
  input.current_messages(1, 0) = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(communication_delay_loss_step(input, output));
}
}  // namespace
