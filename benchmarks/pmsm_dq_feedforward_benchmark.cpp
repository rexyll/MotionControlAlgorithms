#include <benchmark/benchmark.h>

#include "mca/control/pmsm_dq_feedforward.hpp"

namespace {

void BM_PmsmDqFeedforwardCompute(benchmark::State& state) {
  mca::control::PmsmDqFeedforwardConfig config;
  config.d_axis_inductance_h = 0.001;
  config.q_axis_inductance_h = 0.0012;
  config.permanent_magnet_flux_wb = 0.05;
  config.voltage_d_min = -24.0;
  config.voltage_d_max = 24.0;
  config.voltage_q_min = -24.0;
  config.voltage_q_max = 24.0;
  mca::control::PmsmDqFeedforward feedforward(config);
  mca::control::PmsmDqFeedforwardOutput output;

  for (auto _ : state) {
    const auto status = feedforward.compute(800.0, -0.5, 2.0, output);
    benchmark::DoNotOptimize(status);
    benchmark::DoNotOptimize(output);
  }
}

BENCHMARK(BM_PmsmDqFeedforwardCompute);

}  // namespace

BENCHMARK_MAIN();
