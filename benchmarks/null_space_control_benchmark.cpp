#include "mca/control/null_space_control.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_NullSpaceControl(benchmark::State& state) {
  Eigen::MatrixXd jacobian(6, 7);
  jacobian.setZero();
  for (Eigen::Index i = 0; i < 6; ++i) {
    jacobian(i, i) = 1.0;
    jacobian(i, 6) = 0.25;
  }
  Eigen::VectorXd secondary(7);
  secondary.setOnes();
  Eigen::MatrixXd projector;
  Eigen::VectorXd velocity;
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        mca::control::NullSpaceControl::compute(
            jacobian, secondary, projector, velocity, 1e-4));
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_NullSpaceControl);
}  // namespace
BENCHMARK_MAIN();
