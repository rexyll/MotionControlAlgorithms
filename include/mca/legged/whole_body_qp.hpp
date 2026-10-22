#pragma once
#include "mca/legged/qp_task_hierarchy.hpp"
#include <cmath>
#include <string>
#include <vector>
namespace mca::legged {
struct WholeBodyQpTask { Eigen::MatrixXd jacobian; Eigen::VectorXd target; double weight{1.0}; };
struct WholeBodyQpInput {
  Eigen::Index variables{0};
  std::vector<WholeBodyQpTask> tasks;
  Eigen::VectorXd lower_bounds, upper_bounds, reference;
  double regularization{1e-6};
  int iterations{120};
  double step_size{0.0};
};
struct WholeBodyQpOutput {
  Eigen::VectorXd solution;
  std::vector<double> task_residual_norms;
  double objective{0.0};
  int iterations{0};
};
// Box-constrained weighted least-squares QP baseline. General contact inequalities
// and floating-base dynamics equalities are deliberately outside this initial scope.
class WholeBodyQp {
 public:
  static bool solve(const WholeBodyQpInput& in, WholeBodyQpOutput& out, std::string* error=nullptr) {
    const auto fail=[error](const char* m){if(error)*error=m;return false;};
    const Eigen::Index n=in.variables;
    if(n<=0) return fail("whole-body QP variable count must be positive");
    if(in.tasks.empty()) return fail("whole-body QP requires at least one task");
    if(!std::isfinite(in.regularization)||in.regularization<=0) return fail("regularization must be positive and finite");
    if(in.iterations<=0||!std::isfinite(in.step_size)) return fail("iteration count and step size are invalid");
    if((in.lower_bounds.size()!=0&&in.lower_bounds.size()!=n)||
       (in.upper_bounds.size()!=0&&in.upper_bounds.size()!=n)||
       (in.reference.size()!=0&&in.reference.size()!=n)) return fail("whole-body QP vector dimensions are inconsistent");
    if((in.lower_bounds.size()!=0&&!in.lower_bounds.allFinite())||
       (in.upper_bounds.size()!=0&&!in.upper_bounds.allFinite())||
       (in.reference.size()!=0&&!in.reference.allFinite())) return fail("whole-body QP bounds/reference must be finite");
    if(in.lower_bounds.size()!=0&&in.upper_bounds.size()!=0&&
       (in.lower_bounds.array()>in.upper_bounds.array()).any()) return fail("lower bounds exceed upper bounds");
    QPTaskHierarchyInput q;
    q.variables=n; q.regularization=1e-12; q.iterations=in.iterations; q.step_size=in.step_size;
    q.lower_bounds=in.lower_bounds; q.upper_bounds=in.upper_bounds;
    for(const auto& t:in.tasks) {
      if(t.jacobian.rows()<=0||t.jacobian.cols()!=n||t.target.size()!=t.jacobian.rows())
        return fail("whole-body QP task dimensions are inconsistent");
      if(!t.jacobian.allFinite()||!t.target.allFinite()||!std::isfinite(t.weight)||t.weight<=0)
        return fail("whole-body QP task values must be finite and weights positive");
      q.levels.push_back({t.jacobian,t.target,t.weight});
    }
    const Eigen::VectorXd ref=in.reference.size()==0?Eigen::VectorXd::Zero(n):in.reference;
    QPTaskLevel posture; posture.matrix=Eigen::MatrixXd::Identity(n,n);
    posture.target=ref; posture.weight=in.regularization; q.levels.push_back(std::move(posture));
    QPTaskHierarchyOutput solved;
    if(!QPTaskHierarchy::solve(q,solved,error)) return false;
    WholeBodyQpOutput candidate; candidate.solution=solved.solution;
    candidate.task_residual_norms.reserve(in.tasks.size());
    candidate.objective=0.5*in.regularization*(candidate.solution-ref).squaredNorm();
    for(const auto& t:in.tasks) {
      const Eigen::VectorXd residual=t.jacobian*candidate.solution-t.target;
      candidate.task_residual_norms.push_back(residual.norm());
      candidate.objective+=0.5*t.weight*residual.squaredNorm();
    }
    if(!std::isfinite(candidate.objective)) return fail("whole-body QP objective is non-finite");
    candidate.iterations=solved.iterations; out=std::move(candidate); return true;
  }
};
} // namespace mca::legged
