#include <benchmark/benchmark.h>

#include "mca/simulation/wheel_robot_disturbance.hpp"

static void BM_WheelRobotDisturbance(benchmark::State& state) {
  mca::simulation::WheelRobotDisturbanceModel model;
  mca::simulation::WheelRobotDisturbanceConfig config;
  config.longitudinal_slip = 0.12;
  config.lateral_slip = 0.2;
  config.yaw_slip = 0.05;
  config.disturbance_x_amplitude = 0.1;
  config.disturbance_x_frequency = 0.7;
  config.disturbance_y_amplitude = 0.05;
  config.disturbance_y_frequency = 1.2;
  config.disturbance_yaw_amplitude = 0.02;
  config.disturbance_yaw_frequency = 0.4;
  model.configure(config);
  mca::simulation::PlanarTwistCommand actual{};
  for (auto _ : state) {
    benchmark::DoNotOptimize(model.compute({1.0, 0.2, 0.3}, 1.25, actual));
    benchmark::DoNotOptimize(actual);
  }
}
BENCHMARK(BM_WheelRobotDisturbance);
BENCHMARK_MAIN();
