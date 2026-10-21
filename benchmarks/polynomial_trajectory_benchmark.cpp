#include <array>
#include <benchmark/benchmark.h>

#include "mca/trajectory/polynomial.hpp"

namespace {
void BM_PolynomialTrajectories(benchmark::State& state) {
  const mca::trajectory::CubicTrajectory cubic(0.0, 0.0, 1.0, 0.0, 2.0);
  const mca::trajectory::QuinticTrajectory quintic(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0);
  constexpr std::array<double, 8> times{0.0, 0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0};
  std::size_t index = 0U;
  for (auto _ : state) {
    benchmark::DoNotOptimize(_);
    benchmark::DoNotOptimize(cubic.sample(times[index]));
    benchmark::DoNotOptimize(quintic.sample(times[index]));
    index = (index + 1U) % times.size();
  }
}
}  // namespace
BENCHMARK(BM_PolynomialTrajectories);
BENCHMARK_MAIN();
