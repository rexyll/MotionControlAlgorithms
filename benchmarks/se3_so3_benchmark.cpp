#include <benchmark/benchmark.h>
#include "mca/geometry/se3_so3.hpp"
using namespace mca::geometry;
static void BM_SE3SO3(benchmark::State& state){const Quaternion q=axis_angle({1,2,3},.7);const Pose3D p{{1,2,3},q};Vec3 x{.3,-.4,.8};for(auto _:state){benchmark::DoNotOptimize(x=transform_point(p,x));benchmark::DoNotOptimize(x=rotate(q,x));}}
BENCHMARK(BM_SE3SO3);
BENCHMARK_MAIN();
