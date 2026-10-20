#include <array>

#include <benchmark/benchmark.h>

#include "mca/trajectory/scalar.hpp"

namespace {

void BM_ScalarTrajectories(benchmark::State& state) {
  const mca::trajectory::StepTrajectory step(2.0, -1.0);
  const mca::trajectory::RampTrajectory ramp(-1.0, 3.0, 2.0);
  const mca::trajectory::SineTrajectory sine(1.5, 2.0, 0.25, 0.1);
  constexpr std::array<double, 8> times{
      -0.1, 0.0, 0.125, 0.25, 0.75, 1.0, 2.0, 3.0};
  std::size_t index = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(_);
    const auto a = step.sample(times[index]);
    const auto b = ramp.sample(times[index]);
    const auto c = sine.sample(times[index]);
    benchmark::DoNotOptimize(a);
    benchmark::DoNotOptimize(b);
    benchmark::DoNotOptimize(c);
    index = (index + 1) % times.size();
  }
}

}  // namespace

BENCHMARK(BM_ScalarTrajectories);
BENCHMARK_MAIN();
