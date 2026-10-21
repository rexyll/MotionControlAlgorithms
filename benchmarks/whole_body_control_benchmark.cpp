#include "mca/legged/whole_body_control.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_WholeBodyControl(benchmark::State& state) {
  mca::legged::WholeBodyControlInput input;
  input.dof = 12;
  input.feedforward_torque = Eigen::VectorXd::Constant(12, 0.1);
  input.secondary_torque = Eigen::VectorXd::Constant(12, -0.02);
  for (int i = 0; i < 4; ++i) {
    mca::legged::WholeBodyTask task;
    task.jacobian = Eigen::MatrixXd::Random(6, 12);
    task.task_wrench = Eigen::VectorXd::Constant(6, 10.0);
    input.tasks.push_back(task);
  }
  Eigen::VectorXd output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        mca::legged::WholeBodyControl::compute(input, output));
    benchmark::DoNotOptimize(output.data());
  }
}
}  // namespace
BENCHMARK(BM_WholeBodyControl);
BENCHMARK_MAIN();
