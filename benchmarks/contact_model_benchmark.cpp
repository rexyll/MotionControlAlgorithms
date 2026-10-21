#include "mca/legged/contact_model.hpp"

#include <benchmark/benchmark.h>

namespace {
void BM_ContactModel(benchmark::State& state) {
  mca::legged::ContactModelParameters parameters;
  mca::legged::ContactPointState contact;
  contact.gap = -0.001;
  contact.velocity_world = {0.2, -0.1, -0.03};
  mca::legged::ContactWrench output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        mca::legged::evaluate_contact(parameters, contact, output));
    benchmark::DoNotOptimize(output.force_world);
  }
}
BENCHMARK(BM_ContactModel);
}  // namespace
BENCHMARK_MAIN();
