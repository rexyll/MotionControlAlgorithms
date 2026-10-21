#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include "mca/dynamics/rigid_body_dynamics.hpp"

static void BM_RigidBodyMassMatrix(benchmark::State& state) {
  using namespace mca::kinematics;
  using namespace mca::dynamics;
  std::vector<DhParameter> dh(6);
  std::vector<LinkInertial> inertias(7);
  for (std::size_t i=0;i<dh.size();++i) {
    dh[i].name="j"+std::to_string(i+1U); dh[i].type=JointType::Revolute;
    dh[i].a=0.2; inertias[i+1U].mass=1.0+0.1*static_cast<double>(i);
    inertias[i+1U].center_of_mass={0.1,0,0};
    inertias[i+1U].inertia_at_com(0,0)=0.01;
    inertias[i+1U].inertia_at_com(1,1)=0.01;
    inertias[i+1U].inertia_at_com(2,2)=0.01;
  }
  RobotModel model;
  if (!DhModelAdapter::build("bench","base",dh,model)) {
    state.SkipWithError("failed to build benchmark model"); return;
  }
  const Eigen::VectorXd q=Eigen::VectorXd::Constant(6,0.2);
  Eigen::MatrixXd mass;
  for (auto _ : state) {
    benchmark::DoNotOptimize(RigidBodyDynamics::mass_matrix(model,inertias,q,mass));
    benchmark::DoNotOptimize(mass.data());
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_RigidBodyMassMatrix);
BENCHMARK_MAIN();
