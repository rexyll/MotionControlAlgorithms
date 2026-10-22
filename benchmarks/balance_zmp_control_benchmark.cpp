#include <benchmark/benchmark.h>
#include "mca/legged/balance_zmp_control.hpp"
static void BM_ZmpEstimate(benchmark::State& state) {
  mca::legged::ZmpInput input; input.center_of_mass_world={0.01,-0.02,0.82};
  input.center_of_mass_acceleration_world={0.2,-0.1,0.0};
  const mca::legged::RectangularSupportRegion support{-0.12,0.12,-0.08,0.08};
  mca::legged::ZmpOutput output;
  for(auto _:state){const bool ok=mca::legged::estimate_zmp(input,support,output);benchmark::DoNotOptimize(ok);benchmark::DoNotOptimize(output);}
}
static void BM_BalanceAcceleration(benchmark::State& state) {
  mca::legged::BalanceControllerInput input; mca::legged::BalanceControllerOutput output;
  for(auto _:state){const bool ok=mca::legged::compute_balance_acceleration(input,output);benchmark::DoNotOptimize(ok);benchmark::DoNotOptimize(output);}
}
BENCHMARK(BM_ZmpEstimate); BENCHMARK(BM_BalanceAcceleration); BENCHMARK_MAIN();
