#include "mca/multi_robot/communication_delay_loss.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_CommunicationDelayLoss(benchmark::State& state) {
  const Eigen::Index agents = static_cast<Eigen::Index>(state.range(0));
  const Eigen::Index dimensions = static_cast<Eigen::Index>(state.range(1));
  mca::multi_robot::CommunicationDelayLossInput input;
  input.current_messages = Eigen::MatrixXd::Random(agents, dimensions);
  input.previous_messages = Eigen::MatrixXd::Random(agents, dimensions);
  input.last_received_aggregates = Eigen::MatrixXd::Zero(agents, dimensions);
  input.adjacency = Eigen::MatrixXd::Zero(agents, agents);
  input.delay_steps = Eigen::MatrixXd::Zero(agents, agents);
  input.packet_delivered = Eigen::MatrixXd::Ones(agents, agents);
  input.packet_delivered.diagonal().setZero();
  for (Eigen::Index i = 0; i < agents; ++i) {
    const Eigen::Index next = (i + 1) % agents;
    const Eigen::Index previous = (i + agents - 1) % agents;
    input.adjacency(i, next) = 1.0;
    input.adjacency(i, previous) = 1.0;
    input.delay_steps(i, next) = 1.0;
    input.packet_delivered(i, previous) = (i % 4 == 0) ? 0.0 : 1.0;
  }
  mca::multi_robot::CommunicationDelayLossOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        mca::multi_robot::communication_delay_loss_step(input, output));
    benchmark::DoNotOptimize(output.received_aggregates.data());
  }
  state.SetItemsProcessed(state.iterations() * agents);
}
}  // namespace
BENCHMARK(BM_CommunicationDelayLoss)->Args({8, 3})->Args({32, 6})->Args({128, 12});
BENCHMARK_MAIN();
