#include "mca/legged/centroidal_dynamics.hpp"

#include <array>
#include <benchmark/benchmark.h>

namespace {
void BM_CentroidalDynamics(benchmark::State& state) {
  mca::legged::CentroidalDynamicsInput input;
  input.total_mass = 35.0;
  input.applied_forces = {
      {{0.25, 0.15, 0.0}, {0.0, 0.0, 100.0}},
      {{0.25, -0.15, 0.0}, {0.0, 0.0, 100.0}},
      {{-0.25, 0.15, 0.0}, {0.0, 0.0, 100.0}},
      {{-0.25, -0.15, 0.0}, {0.0, 0.0, 100.0}}};
  mca::legged::CentroidalDynamicsOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        mca::legged::evaluate_centroidal_dynamics(input, output));
    benchmark::DoNotOptimize(output.center_of_mass_acceleration_world);
    benchmark::DoNotOptimize(output.angular_momentum_rate_world);
  }
}
}  // namespace
BENCHMARK(BM_CentroidalDynamics);
BENCHMARK_MAIN();
