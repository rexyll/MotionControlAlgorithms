#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include "mca/control/discrete_lqr.hpp"

static void BM_DiscreteLqr(benchmark::State& state) {
  const Eigen::Index n=static_cast<Eigen::Index>(state.range(0));
  Eigen::MatrixXd A=Eigen::MatrixXd::Identity(n,n);
  A.diagonal().array() += 0.01;
  Eigen::MatrixXd B=Eigen::MatrixXd::Random(n,2)*0.1;
  Eigen::MatrixXd Q=Eigen::MatrixXd::Identity(n,n), R=Eigen::MatrixXd::Identity(2,2);
  mca::control::DiscreteLqrResult result;
  for (auto _ : state) {
    const bool ok=mca::control::solve_discrete_lqr(A,B,Q,R,result);
    benchmark::DoNotOptimize(ok);
    benchmark::DoNotOptimize(result.K.data());
  }
  state.SetComplexityN(n);
}
BENCHMARK(BM_DiscreteLqr)->Arg(2)->Arg(4)->Arg(8)->Complexity();
BENCHMARK_MAIN();
