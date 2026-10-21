#include <benchmark/benchmark.h>
#include "mca/kinematics/numerical_ik.hpp"

static void BM_NumericalIkSingleJoint(benchmark::State& state) {
  using namespace mca::kinematics;
  DhParameter p; p.name="j"; p.type=JointType::Revolute; p.a=1.0;
  RobotModel model;
  if (!DhModelAdapter::build("one","base",{p},model)) return;
  ForwardKinematicsResult target;
  if (!ForwardKinematics::solve(model,{0.7},target)) return;
  NumericalIkOptions options;
  for (auto _ : state) {
    const auto result=NumericalIk::solve(model,target.pose,{0.1},options);
    benchmark::DoNotOptimize(result.status);
    benchmark::DoNotOptimize(result.joint_positions);
  }
}
BENCHMARK(BM_NumericalIkSingleJoint);
BENCHMARK_MAIN();
