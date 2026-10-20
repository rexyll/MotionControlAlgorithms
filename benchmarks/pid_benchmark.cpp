#include <benchmark/benchmark.h>

#include "mca/control/pid.hpp"

namespace {

static void BM_PIDUpdate(benchmark::State& state) {
  mca::control::PidConfig config;
  config.kp = 20.0;
  config.ki = 40.0;
  config.kd = 1.0;
  config.output_min = -100.0;
  config.output_max = 100.0;
  config.integral_min = -20.0;
  config.integral_max = 20.0;
  mca::control::PidController pid(config);
  double output = 0.0;

  for (auto _ : state) {
    benchmark::DoNotOptimize(pid.update(1.0, 0.75, 0.001, output));
    benchmark::DoNotOptimize(output);
  }
}

static void BM_PIUpdate(benchmark::State& state) {
  mca::control::PidConfig config;
  config.kp = 20.0;
  config.ki = 40.0;
  config.output_min = -100.0;
  config.output_max = 100.0;
  config.integral_min = -20.0;
  config.integral_max = 20.0;
  mca::control::PidController pi(config);
  double output = 0.0;

  for (auto _ : state) {
    benchmark::DoNotOptimize(pi.update(1.0, 0.75, 0.001, output));
    benchmark::DoNotOptimize(output);
  }
}

static void BM_PDUpdate(benchmark::State& state) {
  mca::control::PidConfig config;
  config.kp = 20.0;
  config.kd = 1.0;
  config.output_min = -100.0;
  config.output_max = 100.0;
  mca::control::PidController pd(config);
  double output = 0.0;

  for (auto _ : state) {
    benchmark::DoNotOptimize(pd.update(1.0, 0.75, 0.001, output));
    benchmark::DoNotOptimize(output);
  }
}

}  // namespace

BENCHMARK(BM_PIDUpdate);
BENCHMARK(BM_PIUpdate);
BENCHMARK(BM_PDUpdate);
BENCHMARK_MAIN();
