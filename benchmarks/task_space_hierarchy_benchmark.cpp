#include "mca/legged/task_space_hierarchy.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_TaskSpaceHierarchy(benchmark::State& state) {
  mca::legged::TaskSpaceHierarchyInput input;
  input.variables = 18;
  for (int i = 0; i < 3; ++i) {
    mca::legged::TaskSpaceLevel level;
    level.jacobian = Eigen::MatrixXd::Constant(6, 18, 0.01 * static_cast<double>(i + 1));
    for (Eigen::Index j = 0; j < level.jacobian.rows(); ++j)
      level.jacobian(j, j + i) = 1.0;
    level.target_velocity = Eigen::VectorXd::Constant(6, 0.1 * static_cast<double>(i + 1));
    input.levels.push_back(level);
  }
  mca::legged::TaskSpaceHierarchyOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(mca::legged::TaskSpaceHierarchy::solve(input, output));
    benchmark::DoNotOptimize(output.velocity.data());
  }
}
}  // namespace
BENCHMARK(BM_TaskSpaceHierarchy);
BENCHMARK_MAIN();
