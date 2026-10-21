#include <benchmark/benchmark.h>
#include "mca/kinematics/damped_least_squares_ik.hpp"

static void BM_DampedLeastSquaresIkSingleJoint(benchmark::State& state) {
  using namespace mca::kinematics;
  DhParameter p; p.name = "j"; p.type = JointType::Revolute; p.a = 1.0;
  RobotModel model;
  if (!DhModelAdapter::build("one", "base", {p}, model)) return;
  ForwardKinematicsResult target;
  if (!ForwardKinematics::solve(model, {0.7}, target)) return;
  for (auto _ : state) {
    const auto result = DampedLeastSquaresIk::solve(model, target.pose, {0.1});
    benchmark::DoNotOptimize(result.status);
    benchmark::DoNotOptimize(result.joint_positions);
  }
}
BENCHMARK(BM_DampedLeastSquaresIkSingleJoint);
BENCHMARK_MAIN();
