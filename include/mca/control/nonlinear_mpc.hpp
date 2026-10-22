#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <string>
#include <utility>
#include <Eigen/Dense>

namespace mca::control {
struct NonlinearMpcOptions {
  std::size_t horizon{10U};
  std::size_t optimization_iterations{40U};
  double gradient_step{0.05};
  double finite_difference_step{1e-5};
};
struct NonlinearMpcProblem {
  std::function<bool(const Eigen::VectorXd&, const Eigen::VectorXd&, Eigen::VectorXd&)> dynamics;
  std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)> stage_cost;
  std::function<double(const Eigen::VectorXd&)> terminal_cost;
  Eigen::VectorXd control_lower;
  Eigen::VectorXd control_upper;
};
struct NonlinearMpcResult {
  Eigen::VectorXd control;
  Eigen::MatrixXd predicted_states;
  Eigen::MatrixXd control_sequence;
  double cost{0.0};
};

// Generic direct-shooting nonlinear MPC using projected finite-difference gradient descent.
inline bool solve_nonlinear_mpc(const NonlinearMpcProblem& problem,
                                const Eigen::VectorXd& initial_state,
                                const NonlinearMpcOptions& options,
                                NonlinearMpcResult& output,
                                std::string* error = nullptr) {
  const auto fail=[error](const char* m){ if(error) *error=m; return false; };
  const Eigen::Index nu=problem.control_lower.size();
  if(!problem.dynamics||!problem.stage_cost||!problem.terminal_cost||initial_state.size()<=0||nu<=0||
     problem.control_upper.size()!=nu||options.horizon==0U||options.horizon>200U||
     options.optimization_iterations==0U||options.optimization_iterations>1000U||
     !std::isfinite(options.gradient_step)||options.gradient_step<=0.0||
     !std::isfinite(options.finite_difference_step)||options.finite_difference_step<=0.0)
    return fail("invalid nonlinear MPC configuration or callbacks");
  if(!initial_state.allFinite()||!problem.control_lower.allFinite()||!problem.control_upper.allFinite()||
     (problem.control_lower.array()>problem.control_upper.array()).any())
    return fail("invalid nonlinear MPC state or control bounds");
  const std::size_t N=options.horizon;
  const Eigen::Index nx=initial_state.size();
  Eigen::MatrixXd controls=Eigen::MatrixXd::Zero(nu,static_cast<Eigen::Index>(N));
  for(Eigen::Index k=0;k<controls.cols();++k) controls.col(k)=0.5*(problem.control_lower+problem.control_upper);
  Eigen::MatrixXd states(nx,static_cast<Eigen::Index>(N+1U));
  Eigen::VectorXd next(nx);
  const auto rollout=[&](const Eigen::MatrixXd& sequence,Eigen::MatrixXd& rollout_states,double& total_cost)->bool{
    rollout_states.col(0)=initial_state; total_cost=0.0;
    for(std::size_t k=0;k<N;++k){
      const Eigen::VectorXd xk=rollout_states.col(static_cast<Eigen::Index>(k));
      const Eigen::VectorXd uk=sequence.col(static_cast<Eigen::Index>(k));
      const double stage=problem.stage_cost(xk,uk);
      if(!std::isfinite(stage)||!problem.dynamics(xk,uk,next)||next.size()!=nx||!next.allFinite()) return false;
      total_cost+=stage; if(!std::isfinite(total_cost)) return false;
      rollout_states.col(static_cast<Eigen::Index>(k+1U))=next;
    }
    const double terminal=problem.terminal_cost(rollout_states.col(static_cast<Eigen::Index>(N)));
    if(!std::isfinite(terminal)) return false; total_cost+=terminal; return std::isfinite(total_cost);
  };
  double current_cost=0.0;
  if(!rollout(controls,states,current_cost)) return fail("initial nonlinear MPC rollout failed");
  Eigen::MatrixXd trial=controls, trial_states=states;
  Eigen::MatrixXd gradient=Eigen::MatrixXd::Zero(nu,static_cast<Eigen::Index>(N));
  for(std::size_t iteration=0;iteration<options.optimization_iterations;++iteration){
    for(std::size_t k=0;k<N;++k) for(Eigen::Index j=0;j<nu;++j){
      const Eigen::Index kk=static_cast<Eigen::Index>(k); const double h=options.finite_difference_step;
      const double base=controls(j,kk);
      const double plus=std::min(problem.control_upper(j),base+h);
      const double minus=std::max(problem.control_lower(j),base-h);
      const double actual_h=0.5*(plus-minus);
      if(actual_h<=0.0||!std::isfinite(actual_h)) return fail("invalid finite-difference step at control bound");
      trial=controls; trial(j,kk)=plus; double plus_cost=0.0;
      const bool plus_ok=rollout(trial,trial_states,plus_cost);
      trial=controls; trial(j,kk)=minus; double minus_cost=0.0;
      const bool minus_ok=rollout(trial,trial_states,minus_cost);
      if(!plus_ok||!minus_ok) return fail("nonlinear MPC finite-difference rollout failed");
      gradient(j,kk)=(plus_cost-minus_cost)/(2.0*actual_h);
    }
    if(!gradient.allFinite()) return fail("nonlinear MPC gradient is non-finite");
    for(Eigen::Index j=0;j<nu;++j) for(std::size_t k=0;k<N;++k){
      const Eigen::Index kk=static_cast<Eigen::Index>(k);
      controls(j,kk)=std::clamp(controls(j,kk)-options.gradient_step*gradient(j,kk),problem.control_lower(j),problem.control_upper(j));
    }
    if(!rollout(controls,states,current_cost)) return fail("nonlinear MPC rollout failed");
  }
  NonlinearMpcResult candidate;
  candidate.control=controls.col(0); candidate.predicted_states=states; candidate.control_sequence=controls; candidate.cost=current_cost;
  if(!candidate.control.allFinite()||!candidate.predicted_states.allFinite()||!candidate.control_sequence.allFinite()||!std::isfinite(candidate.cost))
    return fail("nonlinear MPC result is non-finite");
  output=std::move(candidate); if(error) error->clear(); return true;
}
} // namespace mca::control
