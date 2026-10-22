#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include "mca/control/nonlinear_mpc.hpp"

namespace {
using namespace mca::control;
NonlinearMpcProblem problem(){
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0); p.control_upper=Eigen::VectorXd::Constant(1,1.0);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd& u,Eigen::VectorXd& next){ next.resize(1); next(0)=x(0)+0.2*std::tanh(u(0)); return true; };
  p.stage_cost=[](const Eigen::VectorXd& x,const Eigen::VectorXd& u){ return x.squaredNorm()+0.01*u.squaredNorm(); };
  p.terminal_cost=[](const Eigen::VectorXd& x){ return 2.0*x.squaredNorm(); }; return p;
}
static void BM_NonlinearMpc(benchmark::State& state){
  const auto p=problem(); NonlinearMpcOptions o; o.horizon=static_cast<std::size_t>(state.range(0)); o.optimization_iterations=8; o.gradient_step=0.12;
  Eigen::VectorXd x(1); x<<1.0;
  for(auto _ : state){ NonlinearMpcResult r; benchmark::DoNotOptimize(solve_nonlinear_mpc(p,x,o,r)); benchmark::DoNotOptimize(r.cost); }
}
}
BENCHMARK(BM_NonlinearMpc)->Arg(4)->Arg(8)->Arg(12);
BENCHMARK_MAIN();
