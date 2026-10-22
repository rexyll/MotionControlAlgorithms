#include "mca/control/linearization.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <string>
namespace {
using mca::control::LinearizationOptions; using mca::control::LinearizationResult;
using mca::control::linearize_discrete_dynamics;
TEST(Linearization, RecoversLinearDiscreteDynamicsAndAffineOffset) {
  Eigen::Matrix2d a; a << 1.0,0.2,-0.1,0.9;
  Eigen::Vector2d b(0.5,-0.25), c(0.1,0.3);
  const auto f=[=](const Eigen::VectorXd& x,const Eigen::VectorXd& u,Eigen::VectorXd& y){y=a*x+b*u(0)+c;return true;};
  Eigen::Vector2d x(0.4,-0.7); Eigen::VectorXd u(1);u<<0.8; LinearizationResult r;
  ASSERT_TRUE(linearize_discrete_dynamics(f,x,u,r));
  EXPECT_TRUE(r.A.isApprox(a,1e-9)); EXPECT_TRUE(r.B.isApprox(b,1e-9));
  EXPECT_TRUE(r.affine_offset.isApprox(c,1e-9)); EXPECT_TRUE(r.operating_next_state.isApprox(a*x+b*u(0)+c,1e-12));
}
TEST(Linearization, CentralDifferenceApproximatesNonlinearDynamics) {
  const auto f=[](const Eigen::VectorXd& x,const Eigen::VectorXd& u,Eigen::VectorXd& y){
    y.resize(2); y(0)=std::sin(x(0))+u(0)*u(0); y(1)=x(0)*x(1)+2.0*u(0); return true;};
  Eigen::Vector2d x(0.3,-0.4); Eigen::VectorXd u(1);u<<0.6; LinearizationResult r;
  ASSERT_TRUE(linearize_discrete_dynamics(f,x,u,r));
  EXPECT_NEAR(r.A(0,0),std::cos(x(0)),1e-8); EXPECT_NEAR(r.A(0,1),0.0,1e-8);
  EXPECT_NEAR(r.A(1,0),x(1),1e-8); EXPECT_NEAR(r.A(1,1),x(0),1e-8);
  EXPECT_NEAR(r.B(0,0),2.0*u(0),1e-8); EXPECT_NEAR(r.B(1,0),2.0,1e-8);
}
TEST(Linearization, RejectsInvalidEvaluatorAndPreservesOutput) {
  LinearizationResult out; out.A=Eigen::MatrixXd::Constant(1,1,42.0);
  Eigen::VectorXd x(1),u(1);x<<1.0;u<<2.0;std::string error;
  EXPECT_FALSE(linearize_discrete_dynamics({},x,u,out,{},&error)); EXPECT_FALSE(error.empty());
  ASSERT_EQ(out.A.rows(),1); EXPECT_DOUBLE_EQ(out.A(0,0),42.0);
  const auto wrong=[](const Eigen::VectorXd&,const Eigen::VectorXd&,Eigen::VectorXd& y){y=Eigen::VectorXd::Zero(2);return true;};
  EXPECT_FALSE(linearize_discrete_dynamics(wrong,x,u,out));
  const auto bad=[](const Eigen::VectorXd&,const Eigen::VectorXd&,Eigen::VectorXd& y){y=Eigen::VectorXd::Constant(1,std::numeric_limits<double>::infinity());return true;};
  EXPECT_FALSE(linearize_discrete_dynamics(bad,x,u,out));
  LinearizationOptions opts;opts.finite_difference_step=0.0;
  const auto identity=[](const Eigen::VectorXd& s,const Eigen::VectorXd&,Eigen::VectorXd& y){y=s;return true;};
  EXPECT_FALSE(linearize_discrete_dynamics(identity,x,u,out,opts)); EXPECT_DOUBLE_EQ(out.A(0,0),42.0);
}
} // namespace
