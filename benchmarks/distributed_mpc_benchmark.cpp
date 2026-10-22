#include "mca/multi_robot/distributed_mpc.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_DistributedMpc(benchmark::State& state) {
  const Eigen::Index agents = static_cast<Eigen::Index>(state.range(0));
  const Eigen::Index horizon = static_cast<Eigen::Index>(state.range(1));
  mca::multi_robot::DistributedMpcInput input;
  input.horizon = horizon;
  input.control_dimensions = 2;
  input.control_sequences = Eigen::MatrixXd::Random(agents, horizon * 2);
  input.local_gradients = Eigen::MatrixXd::Random(agents, horizon * 2);
  input.adjacency = Eigen::MatrixXd::Zero(agents, agents);
  for (Eigen::Index i = 0; i + 1 < agents; ++i) {
    input.adjacency(i, i + 1) = 1.0;
    input.adjacency(i + 1, i) = 1.0;
  }
  mca::multi_robot::DistributedMpcOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        mca::multi_robot::distributed_mpc_step(input, output));
    benchmark::DoNotOptimize(output.first_controls.data());
  }
  state.SetItemsProcessed(state.iterations() * agents);
}
}  // namespace
BENCHMARK(BM_DistributedMpc)->Args({4, 10})->Args({16, 20})->Args({64, 30});
BENCHMARK_MAIN();
