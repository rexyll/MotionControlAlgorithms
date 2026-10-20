#include <benchmark/benchmark.h>

#include "mca/control/disturbance_observer.hpp"

static void BM_DisturbanceObserver(benchmark::State& state) {
  mca::control::DisturbanceObserverConfig config;
  config.inertia_kg_m2 = 0.01;
  config.viscous_damping_nm_per_rad_s = 0.1;
  config.filter_time_constant_s = 0.001;
  mca::control::DisturbanceObserver observer(config);
  mca::control::DisturbanceObserverOutput output;
  double velocity = 0.0;
  for (auto _ : state) {
    velocity += 0.01;
    benchmark::DoNotOptimize(observer.update(0.5, velocity, 0.0001, output));
    benchmark::DoNotOptimize(output);
  }
}

BENCHMARK(BM_DisturbanceObserver);
BENCHMARK_MAIN();
