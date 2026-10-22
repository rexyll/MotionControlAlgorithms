#include <benchmark/benchmark.h>
#include "mca/control/linearization.hpp"
namespace {
void BM_LinearizeNonlinearDynamics(benchmark::State& state) {
 const Eigen::Index n=static_cast<Eigen::Index>(state.range(0)), m=static_cast<Eigen::Index>(state.range(1));
 const mca::control::DiscreteDynamicsEvaluator f=[n,m](const Eigen::VectorXd& x,const Eigen::VectorXd& u,Eigen::VectorXd& y){
  y.resize(n); for(Eigen::Index i=0;i<n;++i){const Eigen::Index j=i%m;y(i)=0.9*x(i)+0.01*std::sin(x(i))+0.1*u(j);}return true;};
 const Eigen::VectorXd x=Eigen::VectorXd::Constant(n,0.2),u=Eigen::VectorXd::Constant(m,0.5);
 mca::control::LinearizationResult r;
 for(auto _:state){benchmark::DoNotOptimize(mca::control::linearize_discrete_dynamics(f,x,u,r));benchmark::DoNotOptimize(r.A.data());}
 state.SetItemsProcessed(state.iterations());
}
}
BENCHMARK(BM_LinearizeNonlinearDynamics)->Args({4,2})->Args({12,3})->Args({32,8});
BENCHMARK_MAIN();
