#include <benchmark/benchmark.h>
#include <cmath>
#include <vector>
#include "mca/kinematics/dh_urdf_model.hpp"
#include "mca/kinematics/forward_kinematics.hpp"

static void BM_ForwardKinematics(benchmark::State& state) {
  std::vector<mca::kinematics::DhParameter> dh(6);
  for (std::size_t i = 0; i < dh.size(); ++i) {
    dh[i].name = "j" + std::to_string(i + 1U);
    dh[i].type = mca::kinematics::JointType::Revolute;
    dh[i].a = 0.2 + 0.01 * static_cast<double>(i);
    dh[i].alpha = (i % 2U == 0U) ? 0.0 : 0.5;
  }
  mca::kinematics::RobotModel model;
  if (!mca::kinematics::DhModelAdapter::build("bench", "base", dh, model)) return;
  const std::vector<double> q{0.1, -0.2, 0.3, -0.1, 0.2, -0.15};
  mca::kinematics::ForwardKinematicsResult result;
  for (auto _ : state) {
    benchmark::DoNotOptimize(mca::kinematics::ForwardKinematics::solve(model, q, result));
    benchmark::DoNotOptimize(result.pose.position.x);
  }
}
BENCHMARK(BM_ForwardKinematics);
BENCHMARK_MAIN();
