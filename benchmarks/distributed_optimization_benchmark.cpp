#include "mca/multi_robot/distributed_optimization.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_DistributedOptimization(benchmark::State& state) {
  const Eigen::Index agents = static_cast<Eigen::Index>(state.range(0));
  mca::multi_robot::DistributedOptimizationInput input;
  input.estimates = Eigen::MatrixXd::Random(agents, 4);
  input.gradients = Eigen::MatrixXd::Random(agents, 4);
  input.gradients_at_next_estimates = Eigen::MatrixXd::Random(agents, 4);
  input.gradient_trackers = Eigen::MatrixXd::Random(agents, 4);
  input.mixing = Eigen::MatrixXd::Constant(
      agents, agents, 1.0 / static_cast<double>(agents));
  mca::multi_robot::DistributedOptimizationOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        mca::multi_robot::distributed_optimization_step(input, output));
    benchmark::DoNotOptimize(output.estimates.data());
  }
  state.SetItemsProcessed(state.iterations() * agents);
}
}  // namespace
BENCHMARK(BM_DistributedOptimization)->Arg(8)->Arg(32)->Arg(128);
BENCHMARK_MAIN();
