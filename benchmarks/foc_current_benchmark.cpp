#include <benchmark/benchmark.h>

#include "mca/control/foc_current.hpp"

namespace {

void BM_FocCurrentUpdate(benchmark::State& state) {
  mca::control::FocCurrentControllerConfig config;
  config.kp_d = 0.8;
  config.ki_d = 80.0;
  config.kp_q = 0.8;
  config.ki_q = 80.0;
  config.current_min = -5.0;
  config.current_max = 5.0;
  config.voltage_d_min = -24.0;
  config.voltage_d_max = 24.0;
  config.voltage_q_min = -24.0;
  config.voltage_q_max = 24.0;
  config.integral_d_min = -24.0;
  config.integral_d_max = 24.0;
  config.integral_q_min = -24.0;
  config.integral_q_max = 24.0;
  mca::control::FocCurrentController controller(config);
  mca::control::FocCurrentControllerOutput output;

  for (auto _ : state) {
    const auto status = controller.update(0.5, 2.0, 0.45, 1.95, 1.0e-5, output);
    benchmark::DoNotOptimize(status);
    benchmark::DoNotOptimize(output);
  }
}
BENCHMARK(BM_FocCurrentUpdate);

}  // namespace

BENCHMARK_MAIN();
