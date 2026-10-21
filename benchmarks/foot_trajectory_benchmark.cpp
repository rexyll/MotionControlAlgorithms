#include "mca/legged/foot_trajectory.hpp"

#include <array>
#include <benchmark/benchmark.h>

namespace {
void BM_FootTrajectory(benchmark::State& state) {
  mca::legged::FootTrajectoryParameters parameters;
  parameters.start_position_world = {0.0, 0.0, 0.1};
  parameters.finish_position_world = {0.4, 0.1, 0.1};
  parameters.swing_height = 0.08;
  parameters.duration_s = 0.6;
  mca::legged::FootTrajectorySample output;
  constexpr std::array<double, 8> times{0.0, 0.05, 0.1, 0.2, 0.3, 0.4, 0.55, 0.6};
  std::size_t index = 0U;
  for (auto _ : state) {
    benchmark::DoNotOptimize(_);
    benchmark::DoNotOptimize(
        mca::legged::evaluate_foot_trajectory(parameters, times[index], output));
    benchmark::DoNotOptimize(output.position_world);
    index = (index + 1U) % times.size();
  }
}
}  // namespace
BENCHMARK(BM_FootTrajectory);
BENCHMARK_MAIN();
