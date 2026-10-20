#include <array>
#include <cmath>

#include <benchmark/benchmark.h>

#include "mca/analysis/step_response_metrics.hpp"

namespace {

void BM_StepResponseMetrics(benchmark::State& state) {
  std::array<double, 1000> time{};
  std::array<double, 1000> value{};
  for (std::size_t i = 0; i < time.size(); ++i) {
    time[i] = static_cast<double>(i) * 0.001;
    const double t = time[i];
    value[i] = 1.0 - 1.2 * std::exp(-5.0 * t) * std::cos(18.0 * t);
  }

  for (auto _ : state) {
    mca::analysis::StepResponseMetrics metrics;
    const auto status = mca::analysis::compute_step_response_metrics(
        time.data(), value.data(), time.size(), 0.0, 1.0, 0.02, 0.05,
        metrics);
    benchmark::DoNotOptimize(status);
    benchmark::DoNotOptimize(metrics);
  }
}

}  // namespace

BENCHMARK(BM_StepResponseMetrics);
BENCHMARK_MAIN();
