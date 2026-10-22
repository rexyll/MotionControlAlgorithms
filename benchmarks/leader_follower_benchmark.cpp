#include "mca/multi_robot/leader_follower.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_LeaderFollower(benchmark::State& state) {
  const Eigen::Index agents = static_cast<Eigen::Index>(state.range(0));
  mca::multi_robot::LeaderFollowerInput input;
  input.positions = Eigen::MatrixXd::Random(agents, 2);
  input.desired_offsets = Eigen::MatrixXd::Random(agents, 2);
  input.leader_index = 0;
  input.leader_velocity = Eigen::Vector2d(0.1, -0.1);
  input.gain = 1.0;
  input.max_follower_speed = 2.0;
  mca::multi_robot::LeaderFollowerOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(mca::multi_robot::leader_follower_control(input, output));
    benchmark::DoNotOptimize(output.velocities.data());
  }
  state.SetItemsProcessed(state.iterations() * agents);
}
}  // namespace
BENCHMARK(BM_LeaderFollower)->Arg(8)->Arg(32)->Arg(128);
BENCHMARK_MAIN();
