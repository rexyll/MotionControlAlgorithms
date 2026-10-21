#include <benchmark/benchmark.h>
#include "mca/kinematics/jacobian.hpp"

static void BM_PlanarTwoLinkJacobian(benchmark::State& state) {
  using namespace mca::kinematics;
  std::vector<DhParameter> dh(2);
  dh[0].name = "j1"; dh[0].type = JointType::Revolute; dh[0].a = 1.0;
  dh[1].name = "j2"; dh[1].type = JointType::Revolute; dh[1].a = 0.5;
  RobotModel model;
  if (!DhModelAdapter::build("planar", "base", dh, model)) {
    state.SkipWithError("failed to build benchmark model");
    return;
  }
  JacobianResult result;
  for (auto _ : state) {
    benchmark::DoNotOptimize(Jacobian::solve(model, {0.4, -0.7}, result));
    benchmark::DoNotOptimize(result.rows);
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_PlanarTwoLinkJacobian);
BENCHMARK_MAIN();
