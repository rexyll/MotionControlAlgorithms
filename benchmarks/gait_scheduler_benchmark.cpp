#include "mca/legged/gait_scheduler.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_GaitScheduler(benchmark::State& state) {
  mca::legged::GaitSchedulerParameters parameters;
  mca::legged::GaitSchedulerOutput output;
  double time_s = 0.0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(_);
    benchmark::DoNotOptimize(mca::legged::evaluate_gait(parameters, time_s, output));
    benchmark::DoNotOptimize(output.legs.data());
    time_s += 0.001;
    if (time_s > parameters.period_s * 100.0) time_s = 0.0;
  }
}
}  // namespace
BENCHMARK(BM_GaitScheduler);
BENCHMARK_MAIN();
