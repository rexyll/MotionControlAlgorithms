#include <cmath>
#include <vector>

#include <benchmark/benchmark.h>

#include "mca/path_tracking/mpc_path_tracker.hpp"

namespace {
void BM_MpcPathTracker(benchmark::State& state) {
  using mca::path_tracking::MpcCommand;
  using mca::path_tracking::MpcConfig;
  using mca::path_tracking::MpcPathPoint2D;
  using mca::path_tracking::MpcPathTracker;
  using mca::path_tracking::MpcPose2D;

  std::vector<MpcPathPoint2D> path;
  path.reserve(501U);
  for (int i = 0; i <= 500; ++i) {
    const double x = 0.05 * static_cast<double>(i);
    path.push_back(MpcPathPoint2D{x, 0.4 * std::sin(0.15 * x)});
  }
  MpcConfig config;
  config.horizon = 12U;
  config.optimization_iterations = 40U;
  MpcPathTracker tracker;
  const bool configured = tracker.configure(config);
  const MpcPose2D pose{4.0, 0.2, 0.05};
  MpcCommand command;
  for (auto _ : state) {
    benchmark::DoNotOptimize(configured);
    benchmark::DoNotOptimize(tracker.compute(pose, path, command));
    benchmark::DoNotOptimize(command);
  }
}
}  // namespace
BENCHMARK(BM_MpcPathTracker);
BENCHMARK_MAIN();
