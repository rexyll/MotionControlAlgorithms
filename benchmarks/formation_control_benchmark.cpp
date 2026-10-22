#include "mca/multi_robot/formation_control.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_FormationControl(benchmark::State& state) {
  const Eigen::Index agents = static_cast<Eigen::Index>(state.range(0));
  mca::multi_robot::FormationInput input;
  input.positions = Eigen::MatrixXd::Random(agents, 2);
  input.desired_offsets = Eigen::MatrixXd::Random(agents, 2);
  input.adjacency = Eigen::MatrixXd::Ones(agents, agents);
  input.adjacency.diagonal().setZero();
  mca::multi_robot::FormationOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(mca::multi_robot::formation_control(input, output));
    benchmark::DoNotOptimize(output.velocities.data());
  }
  state.SetItemsProcessed(state.iterations() * agents);
}
}  // namespace
BENCHMARK(BM_FormationControl)->Arg(8)->Arg(32)->Arg(128);
BENCHMARK_MAIN();
