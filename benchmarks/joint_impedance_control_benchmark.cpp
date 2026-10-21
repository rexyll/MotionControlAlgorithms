#include <benchmark/benchmark.h>

#include <Eigen/Dense>

#include "mca/legged/joint_impedance_control.hpp"

static void BM_JointImpedanceControl(benchmark::State& state) {
  const Eigen::Index n = state.range(0);
  mca::legged::JointImpedanceControlInput input;
  input.position = Eigen::VectorXd::Constant(n, 0.1);
  input.velocity = Eigen::VectorXd::Constant(n, -0.2);
  input.desired_position = Eigen::VectorXd::Constant(n, 0.5);
  input.desired_velocity = Eigen::VectorXd::Zero(n);
  input.stiffness = Eigen::VectorXd::Constant(n, 80.0);
  input.damping = Eigen::VectorXd::Constant(n, 8.0);
  input.feedforward_torque = Eigen::VectorXd::Constant(n, 1.0);
  input.torque_limits = Eigen::VectorXd::Constant(n, 100.0);
  Eigen::VectorXd output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        mca::legged::JointImpedanceControl::compute(input, output));
    benchmark::ClobberMemory();
  }
}

BENCHMARK(BM_JointImpedanceControl)->Arg(12)->Arg(24);
BENCHMARK_MAIN();
