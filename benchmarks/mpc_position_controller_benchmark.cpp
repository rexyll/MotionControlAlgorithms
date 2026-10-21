#include <benchmark/benchmark.h>

#include "mca/uav/mpc_position_controller.hpp"

namespace {
void BM_MpcPositionController(benchmark::State& state) {
  using mca::uav::MpcPositionController;
  using mca::uav::MpcPositionControllerConfig;
  using mca::uav::MpcPositionControllerInput;
  using mca::uav::MpcPositionControllerOutput;

  MpcPositionController controller;
  const auto configured = controller.configure(MpcPositionControllerConfig{});
  MpcPositionControllerInput input;
  input.position_world = {1.0, -0.5, 0.2};
  input.velocity_world = {0.1, -0.2, 0.05};
  MpcPositionControllerOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(configured.ok());
    benchmark::DoNotOptimize(controller.update(input, output).ok());
    benchmark::DoNotOptimize(output);
  }
}
}  // namespace
BENCHMARK(BM_MpcPositionController);
BENCHMARK_MAIN();
