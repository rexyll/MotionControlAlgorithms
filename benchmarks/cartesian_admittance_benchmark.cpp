#include <benchmark/benchmark.h>

#include <Eigen/Dense>

#include "mca/control/cartesian_admittance.hpp"

namespace {

mca::geometry::Pose3D make_pose() {
  mca::geometry::Pose3D pose;
  pose.position = {0.0, 0.0, 0.0};
  pose.rotation = {1.0, 0.0, 0.0, 0.0};
  return pose;
}

static void BM_CartesianAdmittance(benchmark::State& state) {
  const Eigen::VectorXd mass = Eigen::VectorXd::Constant(6, 1.0);
  const Eigen::VectorXd damping = Eigen::VectorXd::Constant(6, 2.0);
  const Eigen::VectorXd stiffness = Eigen::VectorXd::Constant(6, 10.0);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
  wrench[0] = 1.0;

  for (auto _ : state) {
    mca::control::CartesianAdmittance controller;
    controller.configure(mass, damping, stiffness, 0.001);
    controller.reset(make_pose());
    mca::geometry::Pose3D out_pose;
    Eigen::VectorXd out_twist, out_acceleration;
    controller.update(make_pose(), zero, wrench, zero, out_pose, out_twist, out_acceleration);
    benchmark::DoNotOptimize(out_pose);
    benchmark::DoNotOptimize(out_twist);
    benchmark::DoNotOptimize(out_acceleration);
  }
}

}  // namespace

BENCHMARK(BM_CartesianAdmittance);
BENCHMARK_MAIN();
