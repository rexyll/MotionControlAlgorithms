#include <cmath>
#include <vector>

#include <benchmark/benchmark.h>

#include "mca/path_tracking/lqr_path_tracker.hpp"

namespace {
void BM_LqrPathTracker(benchmark::State& state) {
  using mca::path_tracking::LqrCommand;
  using mca::path_tracking::LqrConfig;
  using mca::path_tracking::LqrPathPoint2D;
  using mca::path_tracking::LqrPathTracker;
  using mca::path_tracking::LqrPose2D;

  std::vector<LqrPathPoint2D> path;
  path.reserve(501);
  for (int i = 0; i <= 500; ++i) {
    const double x = 0.05 * static_cast<double>(i);
    path.push_back(LqrPathPoint2D{x, 0.4 * std::sin(0.15 * x)});
  }
  LqrPathTracker tracker;
  const bool configured =
      tracker.configure(LqrConfig{2.0, 1.0, 1.0, 0.05, 1.0, 0.5, 0.6});
  const LqrPose2D pose{4.0, 0.2, 0.05};
  LqrCommand command;
  for (auto _ : state) {
    benchmark::DoNotOptimize(configured);
    benchmark::DoNotOptimize(tracker.compute(pose, path, command));
    benchmark::DoNotOptimize(command);
  }
}
}  // namespace
BENCHMARK(BM_LqrPathTracker);
BENCHMARK_MAIN();
