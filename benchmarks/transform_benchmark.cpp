#include <cmath>

#include <benchmark/benchmark.h>

#include "mca/transform/clarke_park.hpp"

namespace {

void BM_Clarke(benchmark::State& state) {
  double phase = 0.0;
  for (auto _ : state) {
    const auto x = mca::transform::clarke(std::cos(phase),
                                          std::cos(phase - 2.0943951023931953),
                                          std::cos(phase + 2.0943951023931953));
    benchmark::DoNotOptimize(x);
    phase += 0.001;
  }
}
BENCHMARK(BM_Clarke);

void BM_Park(benchmark::State& state) {
  double phase = 0.0;
  const mca::transform::AlphaBeta input{0.8, -0.3};
  for (auto _ : state) {
    const auto x = mca::transform::park(input, phase);
    benchmark::DoNotOptimize(x);
    phase += 0.001;
  }
}
BENCHMARK(BM_Park);

}  // namespace

BENCHMARK_MAIN();
