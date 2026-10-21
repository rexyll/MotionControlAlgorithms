#include <cmath>
#include <vector>

#include <benchmark/benchmark.h>

#include "mca/path_tracking/pure_pursuit.hpp"

namespace {

void BM_PurePursuit(benchmark::State& state) {
  using mca::path_tracking::PathPoint2D;
  using mca::path_tracking::Pose2D;
  using mca::path_tracking::PurePursuit;
  using mca::path_tracking::PurePursuitCommand;
  using mca::path_tracking::PurePursuitConfig;

  std::vector<PathPoint2D> path;
  path.reserve(201);
  for (int i = 0; i <= 200; ++i) {
    const double x = 0.05 * static_cast<double>(i);
    path.push_back(PathPoint2D{x, 0.5 * std::sin(0.2 * x)});
  }

  PurePursuit controller;
  controller.configure(PurePursuitConfig{0.8, 1.0});
  Pose2D pose{2.0, 0.2, 0.05};
  PurePursuitCommand command;

  for (auto _ : state) {
    benchmark::DoNotOptimize(controller.compute(pose, path, command));
    benchmark::DoNotOptimize(command);
  }
}

}  // namespace

BENCHMARK(BM_PurePursuit);
BENCHMARK_MAIN();
