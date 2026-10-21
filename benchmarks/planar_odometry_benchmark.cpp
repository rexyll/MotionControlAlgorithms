#include <benchmark/benchmark.h>

#include "mca/odometry/planar_odometry.hpp"

static void BM_PlanarOdometry(benchmark::State& state) {
  mca::odometry::PlanarOdometry odometry;
  for (auto _ : state) {
    benchmark::DoNotOptimize(odometry.update({1.0, 0.2, 0.5}, 0.001));
    benchmark::DoNotOptimize(odometry.pose());
  }
}
BENCHMARK(BM_PlanarOdometry);

BENCHMARK_MAIN();
