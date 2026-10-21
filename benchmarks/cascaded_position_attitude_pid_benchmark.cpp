#include "mca/uav/cascaded_position_attitude_pid.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_CascadedPositionAttitudePid(benchmark::State& state) {
  mca::uav::CascadedPositionAttitudePidConfig config;
  config.mass = 1.8;
  for (auto& pid : config.position_pid) {
    pid.kp = 2.0;
    pid.ki = 0.1;
    pid.output_min = -5.0;
    pid.output_max = 5.0;
  }
  for (auto& pid : config.attitude_pid) {
    pid.kp = 3.0;
    pid.kd = 0.05;
    pid.output_min = -2.0;
    pid.output_max = 2.0;
  }
  mca::uav::CascadedPositionAttitudePid controller;
  if (!controller.configure(config).ok()) {
    state.SkipWithError("controller configuration failed");
    return;
  }
  mca::uav::CascadedPositionAttitudePidInput input;
  input.position_setpoint_world = {1.0, -0.5, 2.0};
  input.attitude_setpoint_body_to_world =
      mca::geometry::axis_angle({0.0, 0.0, 1.0}, 0.3);
  mca::uav::CascadedPositionAttitudePidOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(controller.update(input, output).ok());
    benchmark::DoNotOptimize(output);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CascadedPositionAttitudePid);
}  // namespace
BENCHMARK_MAIN();
