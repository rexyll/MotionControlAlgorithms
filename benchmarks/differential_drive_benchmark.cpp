#include <benchmark/benchmark.h>

#include "mca/kinematics/differential_drive.hpp"

namespace {

static void BM_DifferentialDriveInverse(benchmark::State& state) {
  const mca::kinematics::DifferentialDriveKinematics kinematics(0.1, 0.5);
  const mca::kinematics::Twist2D twist{1.0, 0.8};
  mca::kinematics::WheelSpeeds wheels{};
  for (auto _ : state) {
    benchmark::DoNotOptimize(kinematics.inverse(twist, wheels));
    benchmark::DoNotOptimize(wheels);
  }
}

static void BM_DifferentialDriveForward(benchmark::State& state) {
  const mca::kinematics::DifferentialDriveKinematics kinematics(0.1, 0.5);
  const mca::kinematics::WheelSpeeds wheels{8.0, 12.0};
  mca::kinematics::Twist2D twist{};
  for (auto _ : state) {
    benchmark::DoNotOptimize(kinematics.forward(wheels, twist));
    benchmark::DoNotOptimize(twist);
  }
}

}  // namespace

BENCHMARK(BM_DifferentialDriveInverse);
BENCHMARK(BM_DifferentialDriveForward);
BENCHMARK_MAIN();
