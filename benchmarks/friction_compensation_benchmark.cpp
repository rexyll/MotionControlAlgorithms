#include <array>

#include <benchmark/benchmark.h>

#include "mca/control/friction_compensation.hpp"

namespace {

void BM_FrictionCompensation(benchmark::State& state) {
  mca::control::FrictionCompensationConfig config;
  config.static_friction_nm = 0.8;
  config.coulomb_friction_nm = 0.5;
  config.viscous_friction_nm_per_rad_s = 0.1;
  config.velocity_deadband_rad_s = 0.2;
  config.compensation_min_nm = -10.0;
  config.compensation_max_nm = 10.0;
  mca::control::FrictionCompensator compensator(config);

  constexpr std::array<double, 8> velocities{
      -4.0, -2.0, -0.1, 0.0, 0.1, 1.0, 2.0, 4.0};
  constexpr std::array<double, 8> requests{
      -2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0, 3.0};

  double output = 0.0;
  std::size_t index = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(_);
    const auto status = compensator.compute(velocities[index], requests[index],
                                            output);
    benchmark::DoNotOptimize(status);
    benchmark::DoNotOptimize(output);
    index = (index + 1) % velocities.size();
  }
}

}  // namespace

BENCHMARK(BM_FrictionCompensation);
BENCHMARK_MAIN();
