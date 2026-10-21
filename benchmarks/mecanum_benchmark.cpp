#include <benchmark/benchmark.h>

#include "mca/kinematics/mecanum.hpp"

namespace {

void BM_MecanumInverse(benchmark::State& state) {
  const mca::kinematics::MecanumKinematics kinematics(0.075, 0.31, 0.27);
  mca::kinematics::MecanumWheelSpeeds wheels{};
  for (auto _ : state) {
    benchmark::DoNotOptimize(kinematics.inverse(0.8, -0.35, 1.2, wheels));
    benchmark::ClobberMemory();
  }
}

void BM_MecanumForward(benchmark::State& state) {
  const mca::kinematics::MecanumKinematics kinematics(0.075, 0.31, 0.27);
  const mca::kinematics::MecanumWheelSpeeds wheels{8.0, 12.0, 10.0, 14.0};
  double linear_x = 0.0;
  double linear_y = 0.0;
  double angular_z = 0.0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        kinematics.forward(wheels, linear_x, linear_y, angular_z));
    benchmark::ClobberMemory();
  }
}

BENCHMARK(BM_MecanumInverse);
BENCHMARK(BM_MecanumForward);

}  // namespace

BENCHMARK_MAIN();
