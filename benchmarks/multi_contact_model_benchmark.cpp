#include <benchmark/benchmark.h>

#include "mca/legged/multi_contact_model.hpp"

namespace {

mca::legged::MultiContactInput make_input(int count) {
  mca::legged::MultiContactInput input;
  input.reference_point_world.setZero();
  for (int i = 0; i < count; ++i) {
    mca::legged::MultiContactPoint point;
    point.position_world = {0.25 * (i % 2 == 0 ? 1.0 : -1.0),
                            0.18 * (i % 4 < 2 ? 1.0 : -1.0), 0.0};
    point.state.gap = -0.001;
    point.state.velocity_world.setZero();
    point.state.plane_normal_world = {0.0, 0.0, 1.0};
    point.parameters.normal_stiffness = 5000.0;
    point.parameters.normal_damping = 50.0;
    point.parameters.friction_coefficient = 0.6;
    point.parameters.tangential_damping = 80.0;
    input.points.push_back(point);
  }
  return input;
}

void BM_MultiContact(benchmark::State& state) {
  const int count = static_cast<int>(state.range(0));
  const auto input = make_input(count);
  mca::legged::MultiContactOutput output;
  for (auto _ : state) {
    const bool ok = mca::legged::MultiContactModel().evaluate(input, &output);
    benchmark::DoNotOptimize(ok);
    benchmark::DoNotOptimize(output.total_force_world);
    benchmark::DoNotOptimize(output.total_torque_world);
  }
}

}  // namespace

BENCHMARK(BM_MultiContact)->Arg(2)->Arg(4)->Arg(8)->Arg(16);
BENCHMARK_MAIN();
