#include "mca/uav/rigid_body_model_6d.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_RigidBodyModel6D(benchmark::State& state) {
  mca::uav::RigidBodyParameters6D parameters;
  parameters.mass = 1.8;
  parameters.inertia_body = {{0.03, 0.0, 0.0,
                              0.0, 0.03, 0.0,
                              0.0, 0.0, 0.05}};
  mca::uav::RigidBodyState6D vehicle;
  vehicle.angular_velocity_body = {0.1, -0.2, 0.15};
  const mca::uav::RigidBodyInput6D input{{0.0, 0.0, 17.658}, {0.01, 0.02, 0.005}};
  mca::uav::RigidBodyDerivative6D derivative;
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        mca::uav::RigidBodyModel6D::derivative(parameters, vehicle, input, derivative));
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_RigidBodyModel6D);
}  // namespace
BENCHMARK_MAIN();
