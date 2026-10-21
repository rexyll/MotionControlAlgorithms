#include <array>
#include <benchmark/benchmark.h>

#include "mca/trajectory/s_curve.hpp"

namespace {
void BM_SCurveTrajectory(benchmark::State& state) {
  const mca::trajectory::SCurveTrajectory trajectory(0.0, 10.0, 1.2, 0.8, 2.0);
  const double duration = trajectory.duration();
  constexpr std::array<double, 8> fractions{0.0, 0.05, 0.15, 0.3, 0.5, 0.7, 0.9, 1.0};
  std::size_t index = 0U;
  for (auto _ : state) {
    benchmark::DoNotOptimize(_);
    benchmark::DoNotOptimize(trajectory.sample(duration * fractions[index]));
    index = (index + 1U) % fractions.size();
  }
}
}  // namespace
BENCHMARK(BM_SCurveTrajectory);
BENCHMARK_MAIN();
