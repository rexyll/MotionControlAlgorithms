#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include <vector>
#include "mca/dynamics/computed_torque.hpp"

static void BM_ComputedTorque(benchmark::State& state) {
  using namespace mca::kinematics;
  using namespace mca::dynamics;
  std::vector<DhParameter> dh(6);
  std::vector<LinkInertial> inertias(7);
  for (std::size_t i = 0; i < dh.size(); ++i) {
    dh[i].name = "j" + std::to_string(i + 1U);
    dh[i].type = JointType::Revolute; dh[i].a = 0.2;
    inertias[i + 1U].mass = 1.0 + 0.1 * static_cast<double>(i);
    inertias[i + 1U].center_of_mass = {0.1, 0, 0};
  }
  RobotModel model;
  if (!DhModelAdapter::build("computed-torque-bench", "base", dh, model)) {
    state.SkipWithError("failed to build benchmark model"); return;
  }
  const Eigen::VectorXd q = Eigen::VectorXd::Constant(6, 0.2);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  const Eigen::VectorXd gains = Eigen::VectorXd::Constant(6, 10.0);
  const Eigen::VectorXd limits = Eigen::VectorXd::Constant(6, 100.0);
  Eigen::VectorXd torque;
  for (auto _ : state) {
    benchmark::DoNotOptimize(ComputedTorque::compute(
        model, inertias, q, zero, q, zero, zero, gains, gains,
        {0, 0, -9.81}, limits, torque));
    benchmark::DoNotOptimize(torque.data());
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_ComputedTorque);
BENCHMARK_MAIN();
