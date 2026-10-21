#include <benchmark/benchmark.h>

#include "mca/uav/lqr_position_controller.hpp"

namespace {
void BM_LqrPositionController(benchmark::State& state) {
  using mca::uav::LqrPositionController;
  using mca::uav::LqrPositionControllerConfig;
  using mca::uav::LqrPositionControllerInput;
  using mca::uav::LqrPositionControllerOutput;

  LqrPositionController controller;
  const auto configured = controller.configure(LqrPositionControllerConfig{});
  LqrPositionControllerInput input;
  input.position_world = {1.0, -0.5, 0.2};
  input.velocity_world = {0.1, -0.2, 0.05};
  input.desired_position_world = {0.0, 0.0, 0.0};
  LqrPositionControllerOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(configured.ok());
    benchmark::DoNotOptimize(controller.update(input, output).ok());
    benchmark::DoNotOptimize(output);
  }
}
}  // namespace
BENCHMARK(BM_LqrPositionController);
BENCHMARK_MAIN();
