#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include <vector>
#include "mca/control/cartesian_control.hpp"

static void BM_CartesianControl(benchmark::State& state) {
  using namespace mca::control;
  using namespace mca::kinematics;
  std::vector<DhParameter> dh(6);
  for (std::size_t i = 0; i < dh.size(); ++i) {
    dh[i].name = "j" + std::to_string(i + 1U);
    dh[i].type = JointType::Revolute; dh[i].a = 0.2;
  }
  RobotModel model;
  if (!DhModelAdapter::build("cartesian-bench", "base", dh, model)) {
    state.SkipWithError("failed to build benchmark model"); return;
  }
  const Eigen::VectorXd q = Eigen::VectorXd::Constant(6, 0.2);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  const Eigen::VectorXd gains = Eigen::VectorXd::Constant(6, 10.0);
  const Eigen::VectorXd limits = Eigen::VectorXd::Constant(6, 100.0);
  geometry::Pose3D desired;
  desired.position = {1.0, 0.1, 0.0};
  Eigen::VectorXd torque;
  for (auto _ : state) {
    benchmark::DoNotOptimize(CartesianControl::compute(
        model, q, zero, desired, zero, zero, gains, gains, limits, torque));
    benchmark::DoNotOptimize(torque.data());
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_CartesianControl);
BENCHMARK_MAIN();
