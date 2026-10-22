#include <benchmark/benchmark.h>

#include "mca/legged/locomotion_mpc.hpp"

namespace {

mca::legged::LocomotionMpcInput make_input(int horizon) {
  mca::legged::LocomotionMpcInput input;
  input.initial_state.setZero();
  input.desired_states = Eigen::MatrixXd::Zero(4, horizon + 1);
  input.dt = 0.05;
  input.state_weight << 4.0, 4.0, 1.0, 1.0;
  input.terminal_weight << 8.0, 8.0, 2.0, 2.0;
  input.control_weight << 0.1, 0.1;
  input.acceleration_limits << 3.0, 3.0;
  input.iterations = 80;
  input.step_size = 0.01;
  for (int k = 0; k <= horizon; ++k) {
    const double t = k * input.dt;
    input.desired_states(0, k) = 0.5 * t * t;
    input.desired_states(2, k) = t;
  }
  return input;
}

void BM_LocomotionMpc(benchmark::State& state) {
  const int horizon = static_cast<int>(state.range(0));
  const auto input = make_input(horizon);
  mca::legged::LocomotionMpcOutput output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(input);
    const bool ok = mca::legged::LocomotionMpc().solve(input, &output);
    benchmark::DoNotOptimize(ok);
    benchmark::DoNotOptimize(output.acceleration);
  }
}

}  // namespace

BENCHMARK(BM_LocomotionMpc)->Arg(10)->Arg(20)->Arg(30);
BENCHMARK_MAIN();
