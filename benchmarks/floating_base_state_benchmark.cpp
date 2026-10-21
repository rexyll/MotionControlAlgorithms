#include "mca/legged/floating_base_state.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_FloatingBaseStateValidation(benchmark::State& state) {
  mca::legged::FloatingBaseState sample;
  sample.joint_positions.resize(12, 0.1);
  sample.joint_velocities.resize(12, 0.2);
  for (auto _ : state) {
    benchmark::DoNotOptimize(mca::legged::validate_floating_base_state(sample));
  }
}
BENCHMARK(BM_FloatingBaseStateValidation);
}  // namespace
BENCHMARK_MAIN();
