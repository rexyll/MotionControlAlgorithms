#include "mca/multi_robot/consensus.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_ConsensusStep(benchmark::State& state) {
  const Eigen::Index agents = static_cast<Eigen::Index>(state.range(0));
  mca::multi_robot::ConsensusInput input;
  input.states = Eigen::MatrixXd::Random(agents, 3);
  input.adjacency = Eigen::MatrixXd::Zero(agents, agents);
  for (Eigen::Index i = 0; i < agents; ++i) {
    const Eigen::Index next = (i + 1) % agents;
    input.adjacency(i, next) = 1.0;
    input.adjacency(i, (i + agents - 1) % agents) = 1.0;
  }
  input.time_step = 0.25;
  mca::multi_robot::ConsensusOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(mca::multi_robot::consensus_step(input, output));
    benchmark::DoNotOptimize(output.states.data());
  }
  state.SetItemsProcessed(state.iterations() * agents);
}
}  // namespace
BENCHMARK(BM_ConsensusStep)->Arg(8)->Arg(32)->Arg(128);
BENCHMARK_MAIN();
