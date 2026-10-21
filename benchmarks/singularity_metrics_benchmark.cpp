#include <benchmark/benchmark.h>
#include "mca/kinematics/singularity_metrics.hpp"

static void BM_SingularityMetricsSixDof(benchmark::State& state) {
  mca::kinematics::JacobianResult j; j.columns = 6U;
  for (auto& row : j.rows) row.assign(6U, 0.0);
  for (std::size_t i = 0; i < 6U; ++i) j.rows[i][i] = static_cast<double>(i + 1U);
  mca::kinematics::SingularityMetricsResult result;
  for (auto _ : state) {
    benchmark::DoNotOptimize(mca::kinematics::SingularityMetrics::solve(j, result));
    benchmark::DoNotOptimize(result.condition_number);
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_SingularityMetricsSixDof);
BENCHMARK_MAIN();
