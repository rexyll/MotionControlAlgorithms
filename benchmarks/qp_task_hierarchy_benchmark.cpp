#include "mca/legged/qp_task_hierarchy.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_QPTaskHierarchy(benchmark::State& state) {
  mca::legged::QPTaskHierarchyInput input;
  input.variables = 12;
  input.iterations = 80;
  for (int i = 0; i < 3; ++i) {
    mca::legged::QPTaskLevel level;
    level.matrix = Eigen::MatrixXd::Random(6, 12);
    level.target = Eigen::VectorXd::Constant(6, 0.25 * static_cast<double>(i + 1));
    level.weight = std::pow(10.0, static_cast<double>(2 - i));
    input.levels.push_back(level);
  }
  input.lower_bounds = Eigen::VectorXd::Constant(12, -2.0);
  input.upper_bounds = Eigen::VectorXd::Constant(12, 2.0);
  mca::legged::QPTaskHierarchyOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(mca::legged::QPTaskHierarchy::solve(input, output));
    benchmark::DoNotOptimize(output.solution.data());
  }
}
}  // namespace
BENCHMARK(BM_QPTaskHierarchy);
BENCHMARK_MAIN();
