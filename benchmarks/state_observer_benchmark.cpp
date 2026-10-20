#include <benchmark/benchmark.h>

#include "mca/control/state_observer.hpp"

namespace {

static void BM_StateObserverUpdate(benchmark::State& state) {
  mca::control::StateObserverConfig config;
  config.inertia_kg_m2 = 0.01;
  config.viscous_damping_nm_per_rad_s = 0.02;
  config.stiffness_nm_per_rad = 0.5;
  config.position_gain_per_s = 100.0;
  config.velocity_gain_per_s2 = 2500.0;

  mca::control::StateObserver observer(config);
  mca::control::StateObserverOutput output;
  double position = 0.0;
  double velocity = 0.0;
  constexpr double dt = 0.001;

  for (auto _ : state) {
    position += velocity * dt;
    velocity += 0.1 * dt;
    observer.update(0.01, position, dt, output);
    benchmark::DoNotOptimize(output);
  }
}

}  // namespace

BENCHMARK(BM_StateObserverUpdate);
BENCHMARK_MAIN();
