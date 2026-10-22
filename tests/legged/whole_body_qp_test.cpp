#include "mca/legged/whole_body_qp.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <string>
namespace {
using namespace mca::legged;
TEST(WholeBodyQp, TracksTask) {
 WholeBodyQpInput in; in.variables=2; in.iterations=200; in.regularization=1e-8;
 WholeBodyQpTask t; t.jacobian=Eigen::Matrix2d::Identity(); t.target=Eigen::Vector2d(1.5,-0.75); in.tasks.push_back(t);
 WholeBodyQpOutput out; ASSERT_TRUE(WholeBodyQp::solve(in,out)); EXPECT_NEAR(out.solution[0],1.5,1e-4); EXPECT_NEAR(out.solution[1],-0.75,1e-4);
 ASSERT_EQ(out.task_residual_norms.size(),1U);
}
TEST(WholeBodyQp, EnforcesVariableBounds) {
 WholeBodyQpInput in; in.variables=2; in.iterations=100; in.lower_bounds=Eigen::Vector2d(-0.2,-1); in.upper_bounds=Eigen::Vector2d(0.4,1);
 WholeBodyQpTask t; t.jacobian=Eigen::Matrix2d::Identity(); t.target=Eigen::Vector2d(2,-0.5); in.tasks.push_back(t);
 WholeBodyQpOutput out; ASSERT_TRUE(WholeBodyQp::solve(in,out)); EXPECT_NEAR(out.solution[0],0.4,1e-8); EXPECT_NEAR(out.solution[1],-0.5,1e-4);
}
TEST(WholeBodyQp, RegularizesTowardReference) {
 WholeBodyQpInput in; in.variables=1; in.regularization=1; in.reference=Eigen::VectorXd::Constant(1,2); in.iterations=100;
 WholeBodyQpTask t; t.jacobian=Eigen::MatrixXd::Ones(1,1); t.target=Eigen::VectorXd::Zero(1); in.tasks.push_back(t);
 WholeBodyQpOutput out; ASSERT_TRUE(WholeBodyQp::solve(in,out)); EXPECT_NEAR(out.solution[0],1,1e-5);
}
TEST(WholeBodyQp, InvalidInputDoesNotMutateOutput) {
 WholeBodyQpInput in; in.variables=2; WholeBodyQpTask t; t.jacobian=Eigen::MatrixXd::Identity(2,2);
 t.target=Eigen::Vector2d(1,std::numeric_limits<double>::quiet_NaN()); in.tasks.push_back(t);
 WholeBodyQpOutput out; out.solution=Eigen::Vector2d(7,8); std::string err;
 EXPECT_FALSE(WholeBodyQp::solve(in,out,&err)); EXPECT_FALSE(err.empty()); EXPECT_DOUBLE_EQ(out.solution[0],7); EXPECT_DOUBLE_EQ(out.solution[1],8);
}
}
