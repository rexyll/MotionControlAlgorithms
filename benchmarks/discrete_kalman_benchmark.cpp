#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include "mca/control/discrete_kalman.hpp"
static void BM_KalmanUpdate(benchmark::State& state) {
  const Eigen::Index n=static_cast<Eigen::Index>(state.range(0));
  Eigen::MatrixXd C=Eigen::MatrixXd::Identity(n,n),R=Eigen::MatrixXd::Identity(n,n)*0.1;
  mca::control::DiscreteKalmanState prior;prior.x=Eigen::VectorXd::Zero(n);prior.P=Eigen::MatrixXd::Identity(n,n);
  Eigen::VectorXd y=Eigen::VectorXd::Ones(n);mca::control::DiscreteKalmanState result;
  for(auto _:state){const bool ok=mca::control::kalman_update(C,R,y,prior,result);benchmark::DoNotOptimize(ok);benchmark::DoNotOptimize(result.x.data());}
}
BENCHMARK(BM_KalmanUpdate)->Arg(2)->Arg(4)->Arg(8);
BENCHMARK_MAIN();
