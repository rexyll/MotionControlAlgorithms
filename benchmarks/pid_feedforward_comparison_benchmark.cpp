#include <benchmark/benchmark.h>

#include "mca/experiment/pid_feedforward_comparison.hpp"

namespace {

mca::experiment::PidFeedforwardComparisonConfig make_config() {
  mca::experiment::PidFeedforwardComparisonConfig config;
  config.dt = 0.001;
  config.steps = 2000;
  config.ramp_target = 1.0;
  config.ramp_duration_s = 1.0;
  config.pid.kp = 10.0;
  config.pid.ki = 0.0;
  config.pid.kd = 4.0;
  config.pid.output_min = -100.0;
  config.pid.output_max = 100.0;
  config.pid.integral_min = -10.0;
  config.pid.integral_max = 10.0;
  config.feedforward.velocity_gain = 0.5;
  config.feedforward.acceleration_gain = 1.0;
  config.plant = mca::plant::SecondOrderPlant(1.0, 0.5, 0.0);
  return config;
}

static void BM_PidFeedforwardComparison(benchmark::State& state) {
  const auto config = make_config();
  for (auto _ : state) {
    mca::experiment::PidFeedforwardComparison runner;
    benchmark::DoNotOptimize(runner.configure(config));
    mca::experiment::PidFeedforwardComparisonResult result;
    benchmark::DoNotOptimize(runner.run(result));
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() * config.steps);
}

}  // namespace

BENCHMARK(BM_PidFeedforwardComparison);
BENCHMARK_MAIN();
