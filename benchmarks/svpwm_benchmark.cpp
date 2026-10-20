#include <cmath>

#include <benchmark/benchmark.h>

#include "mca/control/svpwm.hpp"

namespace {

void BM_SvpwmCompute(benchmark::State& state) {
  mca::control::Svpwm controller({48.0, 0.0001});
  mca::control::SvpwmOutput output;
  double angle = 0.0;
  for (auto _ : state) {
    const mca::control::SvpwmInput input{
        20.0 * std::cos(angle), 20.0 * std::sin(angle)};
    benchmark::DoNotOptimize(controller.compute(input, output));
    benchmark::DoNotOptimize(output);
    angle += 0.017;
    if (angle > 6.283185307179586) angle -= 6.283185307179586;
  }
  state.SetItemsProcessed(state.iterations());
}

}  // namespace

BENCHMARK(BM_SvpwmCompute);
BENCHMARK_MAIN();
