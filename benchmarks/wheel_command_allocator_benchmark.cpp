#include <array>

#include <benchmark/benchmark.h>

#include "mca/kinematics/wheel_command_allocator.hpp"

namespace {

static void BM_WheelCommandAllocator4(benchmark::State& state) {
  const mca::kinematics::WheelCommandAllocator<4> allocator;
  const std::array<double, 4> requested{12.0, -8.0, 6.0, -10.0};
  const std::array<double, 4> limits{10.0, 10.0, 8.0, 9.0};
  std::array<double, 4> allocated{};
  double scale = 0.0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(allocator.allocate(requested, limits, allocated, scale));
    benchmark::DoNotOptimize(allocated);
    benchmark::DoNotOptimize(scale);
  }
}

}  // namespace

BENCHMARK(BM_WheelCommandAllocator4);
BENCHMARK_MAIN();
