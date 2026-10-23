#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include "mca/control/nonlinear_mpc.hpp"
#include "mca/control/constrained_qp.hpp"

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

static void BM_NonlinearMpcWarmStart(benchmark::State& state){
  const auto p=problem();
  NonlinearMpcOptions o; o.horizon=static_cast<std::size_t>(state.range(0)); o.optimization_iterations=8; o.gradient_step=0.12;
  Eigen::VectorXd x(1); x<<1.0;
  Eigen::MatrixXd warm=Eigen::MatrixXd::Zero(1,static_cast<Eigen::Index>(o.horizon));
  for(auto _ : state){
    o.initial_control_sequence=warm;
    NonlinearMpcResult r;
    const bool solved=solve_nonlinear_mpc(p,x,o,r);
    benchmark::DoNotOptimize(solved);
    benchmark::DoNotOptimize(r.cost);
    if(solved){
      Eigen::MatrixXd shifted;
      if(shift_nonlinear_mpc_warm_start(r.control_sequence,o.horizon,shifted)) warm=std::move(shifted);
    }
  }
}

static void BM_ConstrainedQp(benchmark::State& state){
  const Eigen::Index n=static_cast<Eigen::Index>(state.range(0));
  ConstrainedQpProblem p;
  p.hessian=2.0*Eigen::MatrixXd::Identity(n,n);
  p.gradient=Eigen::VectorXd::Constant(n,-1.0);
  p.lower_bound=Eigen::VectorXd::Constant(n,-1.0);
  p.upper_bound=Eigen::VectorXd::Constant(n,1.0);
  ConstrainedQpOptions o; o.max_iterations=100U;
  for(auto _ : state){ ConstrainedQpResult r; benchmark::DoNotOptimize(solve_constrained_qp(p,o,r)); benchmark::DoNotOptimize(r.objective); }
}
}
BENCHMARK(BM_NonlinearMpc)->Arg(4)->Arg(8)->Arg(12);
BENCHMARK(BM_NonlinearMpcWarmStart)->Arg(4)->Arg(8)->Arg(12);
BENCHMARK(BM_ConstrainedQp)->Arg(4)->Arg(8)->Arg(16);
BENCHMARK_MAIN();
