#include <benchmark/benchmark.h>
#include "mca/legged/contact_wrench_optimizer.hpp"
namespace {
mca::legged::ContactWrenchOptimizationInput make_input(int count) {
  mca::legged::ContactWrenchOptimizationInput input;
  input.desired_wrench_world << 0.0, 0.0, 600.0, 0.0, 0.0, 0.0;
  input.iterations = 100;
  for (int i = 0; i < count; ++i) {
    input.contact_positions_world.emplace_back(i % 2 == 0 ? -0.3 : 0.3,
                                                i % 4 < 2 ? -0.2 : 0.2, 0.0);
    input.friction_coefficients.push_back(0.6);
  }
  return input;
}
void BM_ContactWrenchOptimization(benchmark::State& state) {
  const auto input = make_input(static_cast<int>(state.range(0)));
  mca::legged::ContactWrenchOptimizationOutput output;
  for (auto _ : state) {
    const bool ok = mca::legged::ContactWrenchOptimizer::solve(input, output);
    benchmark::DoNotOptimize(ok);
    benchmark::DoNotOptimize(output.achieved_wrench_world);
    benchmark::DoNotOptimize(output.residual_norm);
  }
}
}
BENCHMARK(BM_ContactWrenchOptimization)->Arg(2)->Arg(4)->Arg(8)->Arg(16);
BENCHMARK_MAIN();
