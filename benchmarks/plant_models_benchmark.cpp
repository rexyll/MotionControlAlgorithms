#include <benchmark/benchmark.h>

#include "mca/plant/first_order.hpp"
#include "mca/plant/second_order.hpp"

namespace {

static void BM_FirstOrderPlant(benchmark::State& state) {
  mca::plant::FirstOrderPlant plant(0.05, 1.0);
  for (auto _ : state) {
    benchmark::DoNotOptimize(plant.step(1.0, 0.001));
    benchmark::DoNotOptimize(plant.output());
  }
}

static void BM_SecondOrderPlant(benchmark::State& state) {
  mca::plant::SecondOrderPlant plant(0.01, 0.1, 1.0);
  for (auto _ : state) {
    benchmark::DoNotOptimize(plant.step(1.0, 0.001));
    benchmark::DoNotOptimize(plant.position());
    benchmark::DoNotOptimize(plant.velocity());
  }
}

}  // namespace

BENCHMARK(BM_FirstOrderPlant);
BENCHMARK(BM_SecondOrderPlant);
BENCHMARK_MAIN();
