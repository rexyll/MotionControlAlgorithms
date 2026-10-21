#include <benchmark/benchmark.h>

#include "mca/uav/quadrotor_mixer.hpp"

namespace {
static void BM_QuadrotorMixer(benchmark::State& state) {
  const mca::uav::QuadrotorMixer mixer;
  const mca::uav::QuadrotorMixerConfig config{};
  if (!mixer.configure(config).ok()) state.SkipWithError("configure failed");
  const mca::uav::QuadrotorWrench requested{16.0, {0.2, -0.12, 0.04}};
  mca::uav::QuadrotorMixerOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(mixer.allocate(requested, output));
    benchmark::DoNotOptimize(output);
  }
}
}
BENCHMARK(BM_QuadrotorMixer);
BENCHMARK_MAIN();
