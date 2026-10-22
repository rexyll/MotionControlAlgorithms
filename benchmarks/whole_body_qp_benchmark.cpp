#include "mca/legged/whole_body_qp.hpp"
#include <benchmark/benchmark.h>
namespace {
void BM_WholeBodyQp(benchmark::State& state) {
 const Eigen::Index n=static_cast<Eigen::Index>(state.range(0)); mca::legged::WholeBodyQpInput in;
 in.variables=n; in.iterations=80; in.lower_bounds=Eigen::VectorXd::Constant(n,-2); in.upper_bounds=Eigen::VectorXd::Constant(n,2);
 for(int i=0;i<3;++i){mca::legged::WholeBodyQpTask t;t.jacobian=Eigen::MatrixXd::Random(6,n);t.target=Eigen::VectorXd::Constant(6,0.1*(i+1));t.weight=1.0/(i+1);in.tasks.push_back(t);}
 mca::legged::WholeBodyQpOutput out; for(auto _:state){benchmark::DoNotOptimize(mca::legged::WholeBodyQp::solve(in,out));benchmark::DoNotOptimize(out.solution.data());}
}
}
BENCHMARK(BM_WholeBodyQp)->Arg(12)->Arg(24)->Arg(36); BENCHMARK_MAIN();
