#pragma once

#include <cmath>
#include <string>
#include <utility>

#include <Eigen/Dense>

namespace mca::multi_robot {

struct CommunicationDelayLossInput {
  // Current and previous state/message published by each sender, one row per agent.
  Eigen::MatrixXd current_messages;
  Eigen::MatrixXd previous_messages;
  // Last aggregate delivered to each receiver, used if every incoming packet drops.
  Eigen::MatrixXd last_received_aggregates;
  // Directed link weights. Zero means there is no link; self-links must be zero.
  Eigen::MatrixXd adjacency;
  // Per-link latency in simulation samples: only 0 (current) or 1 (previous) is supported.
  Eigen::MatrixXd delay_steps;
  // Deterministic packet delivery mask: 1 delivered, 0 dropped, for each directed link.
  Eigen::MatrixXd packet_delivered;
};

struct CommunicationDelayLossOutput {
  // Weighted average of the messages delivered to each receiver this sample.
  Eigen::MatrixXd received_aggregates;
  // Number of delivered incoming messages per receiver.
  Eigen::VectorXi fresh_message_count;
  // 1 if all incoming packets were dropped and the last aggregate was held; otherwise 0.
  Eigen::VectorXi held_last_value;
};

// Deterministic communication-channel primitive. A dropped packet is omitted from
// the current weighted average; if no incoming packet arrives, the receiver holds
// its last aggregate. Callers supply masks and history explicitly for replayability.
inline bool communication_delay_loss_step(
    const CommunicationDelayLossInput& input,
    CommunicationDelayLossOutput& output,
    std::string* error = nullptr) {
  const auto fail = [error](const char* message) {
    if (error) *error = message;
    return false;
  };
  const Eigen::Index agents = input.current_messages.rows();
  const Eigen::Index dimensions = input.current_messages.cols();
  if (agents < 2 || dimensions <= 0)
    return fail("messages must contain at least two agents and one dimension");
  if (input.previous_messages.rows() != agents ||
      input.previous_messages.cols() != dimensions ||
      input.last_received_aggregates.rows() != agents ||
      input.last_received_aggregates.cols() != dimensions)
    return fail("current, previous and last-received message dimensions must match");
  if (input.adjacency.rows() != agents || input.adjacency.cols() != agents ||
      input.delay_steps.rows() != agents || input.delay_steps.cols() != agents ||
      input.packet_delivered.rows() != agents ||
      input.packet_delivered.cols() != agents)
    return fail("adjacency, delay_steps and packet_delivered must be square per-agent matrices");
  if (!input.current_messages.allFinite() || !input.previous_messages.allFinite() ||
      !input.last_received_aggregates.allFinite() || !input.adjacency.allFinite() ||
      !input.delay_steps.allFinite() || !input.packet_delivered.allFinite())
    return fail("all message, topology and link-state values must be finite");
  if ((input.adjacency.array() < 0.0).any())
    return fail("adjacency weights must be nonnegative");
  if (input.adjacency.diagonal().cwiseAbs().maxCoeff() > 1e-9)
    return fail("self-links are not supported; adjacency diagonal must be zero");
  for (Eigen::Index receiver = 0; receiver < agents; ++receiver) {
    for (Eigen::Index sender = 0; sender < agents; ++sender) {
      const double delay = input.delay_steps(receiver, sender);
      const double delivered = input.packet_delivered(receiver, sender);
      if (delay != 0.0 && delay != 1.0)
        return fail("delay_steps entries must be 0 or 1");
      if (delivered != 0.0 && delivered != 1.0)
        return fail("packet_delivered entries must be 0 or 1");
    }
  }

  CommunicationDelayLossOutput candidate;
  candidate.received_aggregates = input.last_received_aggregates;
  candidate.fresh_message_count = Eigen::VectorXi::Zero(agents);
  candidate.held_last_value = Eigen::VectorXi::Ones(agents);
  for (Eigen::Index receiver = 0; receiver < agents; ++receiver) {
    Eigen::RowVectorXd weighted_sum = Eigen::RowVectorXd::Zero(dimensions);
    double total_weight = 0.0;
    for (Eigen::Index sender = 0; sender < agents; ++sender) {
      const double weight = input.adjacency(receiver, sender);
      if (weight <= 0.0 || input.packet_delivered(receiver, sender) == 0.0)
        continue;
      const Eigen::MatrixXd& source = input.delay_steps(receiver, sender) == 0.0
          ? input.current_messages : input.previous_messages;
      weighted_sum += weight * source.row(sender);
      total_weight += weight;
      ++candidate.fresh_message_count(receiver);
    }
    if (total_weight > 0.0) {
      candidate.received_aggregates.row(receiver) = weighted_sum / total_weight;
      candidate.held_last_value(receiver) = 0;
    }
  }
  if (!candidate.received_aggregates.allFinite())
    return fail("received aggregates contain non-finite values");
  output = std::move(candidate);
  if (error) error->clear();
  return true;
}

}  // namespace mca::multi_robot
