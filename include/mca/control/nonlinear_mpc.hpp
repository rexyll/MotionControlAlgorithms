#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <Eigen/Dense>

namespace mca::control {
struct NonlinearMpcOptions {
  std::size_t horizon{10U};
  std::size_t optimization_iterations{40U};
  double gradient_step{0.05};
  double finite_difference_step{1e-5};
  std::size_t line_search_iterations{12U};
  // Disabled when zero; otherwise stop when the projected-gradient mapping norm is within tolerance.
  double projected_gradient_tolerance{0.0};
  Eigen::MatrixXd initial_control_sequence;
};
struct NonlinearMpcProblem {
  std::function<bool(const Eigen::VectorXd&, const Eigen::VectorXd&, Eigen::VectorXd&)> dynamics;
  std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)> stage_cost;
  std::function<double(const Eigen::VectorXd&)> terminal_cost;
  Eigen::VectorXd control_lower;
  Eigen::VectorXd control_upper;
};

// Shift a previous receding-horizon control sequence by one step. The last
// available control is repeated when the requested horizon is longer. Output
// is updated only after all inputs have been validated.
inline bool shift_nonlinear_mpc_warm_start(const Eigen::MatrixXd& previous_sequence,
                                           std::size_t new_horizon,
                                           Eigen::MatrixXd& output,
                                           std::string* error = nullptr) {
  const auto fail=[error](const char* message){ if(error) *error=message; return false; };
  if(previous_sequence.rows()<=0||previous_sequence.cols()<=0||
     new_horizon==0U||new_horizon>200U||!previous_sequence.allFinite())
    return fail("invalid nonlinear MPC warm-start sequence or horizon");
  Eigen::MatrixXd candidate(previous_sequence.rows(),static_cast<Eigen::Index>(new_horizon));
  for(std::size_t k=0;k<new_horizon;++k) {
    const Eigen::Index source=std::min(static_cast<Eigen::Index>(k+1U),previous_sequence.cols()-1);
    candidate.col(static_cast<Eigen::Index>(k))=previous_sequence.col(source);
  }
  output=std::move(candidate);
  if(error) error->clear();
  return true;
}

enum class NonlinearMpcTermination {
  IterationLimitReached,
  NoImprovingStep,
  ProjectedGradientToleranceSatisfied
};

struct NonlinearMpcResult {
  Eigen::VectorXd control;
  Eigen::MatrixXd predicted_states;
  Eigen::MatrixXd control_sequence;
  double cost{0.0};
  double projected_gradient_norm{0.0};
  std::size_t iterations_completed{0U};
  std::size_t line_search_evaluations{0U};
  NonlinearMpcTermination termination{NonlinearMpcTermination::IterationLimitReached};
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
     !std::isfinite(options.finite_difference_step)||options.finite_difference_step<=0.0||
     options.line_search_iterations==0U||options.line_search_iterations>100U||
     !std::isfinite(options.projected_gradient_tolerance)||options.projected_gradient_tolerance<0.0)
    return fail("invalid nonlinear MPC configuration or callbacks");
  if(!initial_state.allFinite()||!problem.control_lower.allFinite()||!problem.control_upper.allFinite()||
     (problem.control_lower.array()>problem.control_upper.array()).any())
    return fail("invalid nonlinear MPC state or control bounds");
  const std::size_t N=options.horizon;
  const Eigen::Index nx=initial_state.size();
  Eigen::MatrixXd controls(nu,static_cast<Eigen::Index>(N));
  const bool has_warm_start=options.initial_control_sequence.rows()!=0||options.initial_control_sequence.cols()!=0;
  if(has_warm_start) {
    if(options.initial_control_sequence.rows()!=nu||
       options.initial_control_sequence.cols()!=static_cast<Eigen::Index>(N)||
       !options.initial_control_sequence.allFinite())
      return fail("invalid nonlinear MPC initial control sequence");
    controls=options.initial_control_sequence;
    for(Eigen::Index j=0;j<nu;++j) for(Eigen::Index k=0;k<controls.cols();++k)
      controls(j,k)=std::clamp(controls(j,k),problem.control_lower(j),problem.control_upper(j));
  } else {
    for(Eigen::Index j=0;j<nu;++j) {
      const double midpoint=0.5*problem.control_lower(j)+0.5*problem.control_upper(j);
      controls.row(j).setConstant(midpoint);
    }
  }
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
  std::size_t iterations_completed=0U;
  std::size_t line_search_evaluations=0U;
  NonlinearMpcTermination termination=NonlinearMpcTermination::IterationLimitReached;
  double projected_gradient_norm=0.0;
  for(std::size_t iteration=0;iteration<options.optimization_iterations;++iteration){
    iterations_completed=iteration+1U;
    for(std::size_t k=0;k<N;++k) for(Eigen::Index j=0;j<nu;++j){
      const Eigen::Index kk=static_cast<Eigen::Index>(k); const double h=options.finite_difference_step;
      const double base=controls(j,kk);
      const double plus=std::min(problem.control_upper(j),base+h);
      const double minus=std::max(problem.control_lower(j),base-h);
      if(plus==minus) { gradient(j,kk)=0.0; continue; }
      trial=controls; trial(j,kk)=plus; double plus_cost=0.0;
      const bool plus_ok=rollout(trial,trial_states,plus_cost);
      trial=controls; trial(j,kk)=minus; double minus_cost=0.0;
      const bool minus_ok=rollout(trial,trial_states,minus_cost);
      if(!plus_ok||!minus_ok) return fail("nonlinear MPC finite-difference rollout failed");
      const double denominator=plus-minus;
      if(!std::isfinite(denominator)||denominator<=0.0) return fail("invalid finite-difference interval");
      const double max_cost=std::numeric_limits<double>::max();
      const bool difference_overflows=
        (minus_cost<0.0&&plus_cost>max_cost+minus_cost)||
        (minus_cost>0.0&&plus_cost< -max_cost+minus_cost);
      // Keep the more accurate difference-first path unless subtracting the
      // finite costs would overflow; only then scale before subtraction.
      gradient(j,kk)=difference_overflows
        ? plus_cost/denominator-minus_cost/denominator
        : (plus_cost-minus_cost)/denominator;
    }
    if(!gradient.allFinite()) return fail("nonlinear MPC gradient is non-finite");
    Eigen::MatrixXd projected_gradient(nu,static_cast<Eigen::Index>(N));
    for(Eigen::Index j=0;j<nu;++j) for(Eigen::Index k=0;k<projected_gradient.cols();++k) {
      const double trial_value=controls(j,k)-options.gradient_step*gradient(j,k);
      const double projected=std::clamp(trial_value,problem.control_lower(j),problem.control_upper(j));
      projected_gradient(j,k)=(controls(j,k)-projected)/options.gradient_step;
    }
    // stableNorm avoids overflowing intermediate squares when the norm itself is representable.
    projected_gradient_norm=projected_gradient.stableNorm();
    if(!std::isfinite(projected_gradient_norm)) return fail("nonlinear MPC projected gradient is non-finite");
    if(options.projected_gradient_tolerance>0.0&&projected_gradient_norm<=options.projected_gradient_tolerance) {
      termination=NonlinearMpcTermination::ProjectedGradientToleranceSatisfied;
      break;
    }
    bool accepted=false;
    double step=options.gradient_step;
    for(std::size_t line_search=0U;line_search<options.line_search_iterations;++line_search){
      ++line_search_evaluations;
      trial=controls;
      for(Eigen::Index j=0;j<nu;++j) for(std::size_t k=0;k<N;++k){
        const Eigen::Index kk=static_cast<Eigen::Index>(k);
        const double direction=gradient(j,kk);
        const double delta=step*direction;
        const double proposed=std::isfinite(delta)
          ? controls(j,kk)-delta
          : (direction>0.0?problem.control_lower(j):problem.control_upper(j));
        trial(j,kk)=std::clamp(proposed,problem.control_lower(j),problem.control_upper(j));
      }
      double candidate_cost=0.0;
      if(rollout(trial,trial_states,candidate_cost)&&candidate_cost<current_cost){
        controls=trial;
        states=trial_states;
        current_cost=candidate_cost;
        accepted=true;
        break;
      }
      step*=0.5;
    }
    if(!accepted) {
      termination=NonlinearMpcTermination::NoImprovingStep;
      break;
    }
  }
  NonlinearMpcResult candidate;
  candidate.control=controls.col(0); candidate.predicted_states=states; candidate.control_sequence=controls; candidate.cost=current_cost;
  candidate.projected_gradient_norm=projected_gradient_norm;
  candidate.iterations_completed=iterations_completed;
  candidate.line_search_evaluations=line_search_evaluations;
  candidate.termination=termination;
  if(!candidate.control.allFinite()||!candidate.predicted_states.allFinite()||!candidate.control_sequence.allFinite()||!std::isfinite(candidate.cost))
    return fail("nonlinear MPC result is non-finite");
  output=std::move(candidate); if(error) error->clear(); return true;
}
} // namespace mca::control
