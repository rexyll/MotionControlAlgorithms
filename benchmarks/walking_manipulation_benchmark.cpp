#include "mca/legged/walking_manipulation.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_WalkingManipulation(benchmark::State& state) {
  const Eigen::Index n = static_cast<Eigen::Index>(state.range(0));
  mca::legged::WalkingManipulationInput input;
  input.variables = n;
  input.locomotion_jacobian = Eigen::MatrixXd::Random(6, n);
  input.locomotion_target = Eigen::VectorXd::Constant(6, 0.05);
  input.manipulation_jacobian = Eigen::MatrixXd::Random(6, n);
  input.manipulation_target = Eigen::VectorXd::Constant(6, 0.1);
  input.lower_bounds = Eigen::VectorXd::Constant(n, -1.0);
  input.upper_bounds = Eigen::VectorXd::Constant(n, 1.0);
  input.iterations = 100;
  mca::legged::WalkingManipulationOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        mca::legged::WalkingManipulationController::solve(input, output));
    benchmark::DoNotOptimize(output.generalized_command.data());
  }
}
}  // namespace
BENCHMARK(BM_WalkingManipulation)->Arg(12)->Arg(24)->Arg(36);
BENCHMARK_MAIN();
