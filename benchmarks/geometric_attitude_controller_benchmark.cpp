#include "mca/uav/geometric_attitude_controller.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_GeometricAttitudeController(benchmark::State& state) {
  mca::uav::GeometricAttitudeControllerConfig config;
  config.inertia_diagonal = {0.03, 0.03, 0.05};
  config.attitude_gain = {4.0, 4.0, 2.0};
  config.angular_rate_gain = {0.2, 0.2, 0.1};
  config.torque_limit = {10.0, 10.0, 5.0};
  mca::uav::GeometricAttitudeController controller;
  if (!controller.configure(config).ok()) {
    state.SkipWithError("controller configuration failed");
    return;
  }
  mca::uav::GeometricAttitudeControllerInput input;
  input.attitude_body_to_world =
      mca::geometry::axis_angle({0.0, 0.0, 1.0}, -0.1);
  input.desired_attitude_body_to_world =
      mca::geometry::axis_angle({0.2, 0.3, 0.9}, 0.4);
  input.angular_velocity_body = {0.1, -0.2, 0.15};
  input.desired_angular_velocity_body = {0.0, 0.0, 0.1};
  mca::uav::GeometricAttitudeControllerOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(controller.update(input, output).ok());
    benchmark::DoNotOptimize(output);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_GeometricAttitudeController);
}  // namespace
BENCHMARK_MAIN();
