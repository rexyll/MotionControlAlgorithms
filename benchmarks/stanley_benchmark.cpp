#include <cmath>
#include <vector>

#include <benchmark/benchmark.h>

#include "mca/path_tracking/stanley.hpp"

namespace {
void BM_Stanley(benchmark::State& state) {
  using mca::path_tracking::Stanley;
  using mca::path_tracking::StanleyCommand;
  using mca::path_tracking::StanleyConfig;
  using mca::path_tracking::StanleyPathPoint2D;
  using mca::path_tracking::StanleyPose2D;

  std::vector<StanleyPathPoint2D> path;
  path.reserve(501);
  for (int i = 0; i <= 500; ++i) {
    const double x = 0.05 * static_cast<double>(i);
    path.push_back(StanleyPathPoint2D{x, 0.4 * std::sin(0.15 * x)});
  }
  Stanley controller;
  const bool configured =
      controller.configure(StanleyConfig{1.0, 1.0, 0.1, 1.0, 0.5, 0.6});
  StanleyPose2D pose{4.0, 0.2, 0.05};
  StanleyCommand command;
  for (auto _ : state) {
    benchmark::DoNotOptimize(configured);
    benchmark::DoNotOptimize(controller.compute(pose, path, command));
    benchmark::DoNotOptimize(command);
  }
}
}  // namespace
BENCHMARK(BM_Stanley);
BENCHMARK_MAIN();
