#include <benchmark/benchmark.h>
#include <string>
#include <vector>
#include "mca/kinematics/dh_urdf_model.hpp"

namespace {
const std::string kUrdf = R"(<robot name="bench"><link name="base"/><link name="l1"/><link name="l2"/><link name="tool"/><joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><origin xyz="0.5 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/></joint><joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/><origin xyz="0.5 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/></joint><joint name="tool" type="fixed"><parent link="l2"/><child link="tool"/></joint></robot>)";

static void BM_UrdfParse(benchmark::State& state) {
  for (auto _ : state) {
    mca::kinematics::RobotModel model;
    benchmark::DoNotOptimize(mca::kinematics::UrdfModelAdapter::parse(kUrdf, model));
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_UrdfParse);

static void BM_DhBuild(benchmark::State& state) {
  const std::vector<mca::kinematics::DhParameter> dh{{"j1", mca::kinematics::JointType::Revolute, 0.5, 0.0, 0.0, 0.0, {}}, {"j2", mca::kinematics::JointType::Revolute, 0.5, 0.0, 0.0, 0.0, {}}};
  for (auto _ : state) {
    mca::kinematics::RobotModel model;
    benchmark::DoNotOptimize(mca::kinematics::DhModelAdapter::build("bench", "base", dh, model));
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_DhBuild);
} // namespace
BENCHMARK_MAIN();
