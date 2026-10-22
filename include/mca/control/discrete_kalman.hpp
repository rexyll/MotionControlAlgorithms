#pragma once
#include <cmath>
#include <string>
#include <utility>
#include <Eigen/Dense>

namespace mca::control {
struct DiscreteKalmanState {
  Eigen::VectorXd x;
  Eigen::MatrixXd P;
};
struct DiscreteKalmanOptions {
  double symmetry_tolerance{1e-9};
  double covariance_tolerance{1e-9};
};

// Predict x[k+1]=A*x[k]+B*u[k]+w, y[k]=C*x[k]+v.
// Q and R are process and measurement noise covariance matrices.
inline bool kalman_predict(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::VectorXd& u, const Eigen::MatrixXd& Q,
    const DiscreteKalmanState& state, DiscreteKalmanState& output,
    const DiscreteKalmanOptions& options = {}, std::string* error = nullptr) {
  const auto fail=[error](const char* m){if(error)*error=m;return false;};
  const Eigen::Index n=A.rows();
  if(n<=0||A.cols()!=n||B.rows()!=n||B.cols()!=u.size()||Q.rows()!=n||Q.cols()!=n||
     state.x.size()!=n||state.P.rows()!=n||state.P.cols()!=n)
    return fail("incompatible or empty prediction dimensions");
  if(!A.allFinite()||!B.allFinite()||!u.allFinite()||!Q.allFinite()||
     !state.x.allFinite()||!state.P.allFinite()||
     !std::isfinite(options.symmetry_tolerance)||options.symmetry_tolerance<0.0||
     !std::isfinite(options.covariance_tolerance)||options.covariance_tolerance<0.0)
    return fail("non-finite input or invalid tolerance");
  if((Q-Q.transpose()).norm()>options.symmetry_tolerance||
     (state.P-state.P.transpose()).norm()>options.symmetry_tolerance)
    return fail("covariance matrices must be symmetric");
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> q_eig(0.5*(Q+Q.transpose()));
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> p_eig(0.5*(state.P+state.P.transpose()));
  if(q_eig.info()!=Eigen::Success||p_eig.info()!=Eigen::Success||
     q_eig.eigenvalues().minCoeff() < -options.covariance_tolerance||
     p_eig.eigenvalues().minCoeff() < -options.covariance_tolerance)
    return fail("covariance matrices must be positive semidefinite");
  DiscreteKalmanState candidate;
  candidate.x=A*state.x+B*u;
  candidate.P=A*state.P*A.transpose()+Q;
  candidate.P=0.5*(candidate.P+candidate.P.transpose());
  if(!candidate.x.allFinite()||!candidate.P.allFinite())return fail("prediction is non-finite");
  output=std::move(candidate); if(error)error->clear(); return true;
}

inline bool kalman_update(const Eigen::MatrixXd& C, const Eigen::MatrixXd& R,
    const Eigen::VectorXd& measurement, const DiscreteKalmanState& predicted,
    DiscreteKalmanState& output, const DiscreteKalmanOptions& options = {},
    std::string* error = nullptr) {
  const auto fail=[error](const char* m){if(error)*error=m;return false;};
  const Eigen::Index n=predicted.x.size(), m=measurement.size();
  if(n<=0||m<=0||C.rows()!=m||C.cols()!=n||R.rows()!=m||R.cols()!=m||
     predicted.P.rows()!=n||predicted.P.cols()!=n)
    return fail("incompatible or empty update dimensions");
  if(!C.allFinite()||!R.allFinite()||!measurement.allFinite()||
     !predicted.x.allFinite()||!predicted.P.allFinite()||
     !std::isfinite(options.symmetry_tolerance)||options.symmetry_tolerance<0.0||
     !std::isfinite(options.covariance_tolerance)||options.covariance_tolerance<0.0)
    return fail("non-finite input or invalid tolerance");
  if((R-R.transpose()).norm()>options.symmetry_tolerance||
     (predicted.P-predicted.P.transpose()).norm()>options.symmetry_tolerance)
    return fail("covariance matrices must be symmetric");
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> r_eig(0.5*(R+R.transpose()));
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> p_eig(0.5*(predicted.P+predicted.P.transpose()));
  if(r_eig.info()!=Eigen::Success||p_eig.info()!=Eigen::Success||
     r_eig.eigenvalues().minCoeff()<=options.covariance_tolerance||
     p_eig.eigenvalues().minCoeff() < -options.covariance_tolerance)
    return fail("measurement covariance must be positive definite and state covariance semidefinite");
  const Eigen::MatrixXd S=C*predicted.P*C.transpose()+R;
  Eigen::LLT<Eigen::MatrixXd> factor(S);
  if(factor.info()!=Eigen::Success)return fail("innovation covariance is not positive definite");
  const Eigen::MatrixXd cross=predicted.P*C.transpose();
  const Eigen::MatrixXd K=factor.solve(cross.transpose()).transpose();
  const Eigen::VectorXd innovation=measurement-C*predicted.x;
  DiscreteKalmanState candidate;
  candidate.x=predicted.x+K*innovation;
  // Joseph form preserves symmetry/positive semidefiniteness better numerically.
  const Eigen::MatrixXd I=Eigen::MatrixXd::Identity(n,n);
  const Eigen::MatrixXd IKC=I-K*C;
  candidate.P=IKC*predicted.P*IKC.transpose()+K*R*K.transpose();
  candidate.P=0.5*(candidate.P+candidate.P.transpose());
  if(!candidate.x.allFinite()||!candidate.P.allFinite())return fail("updated estimate is non-finite");
  output=std::move(candidate);if(error)error->clear();return true;
}

// LQG certainty-equivalent feedback: use the Kalman estimate in u = -K*x_hat.
inline bool lqg_control(const Eigen::MatrixXd& K, const Eigen::VectorXd& estimated_state,
                        Eigen::VectorXd& control, std::string* error=nullptr) {
  if(K.cols()!=estimated_state.size()||K.rows()<=0||K.cols()<=0) {
    if(error)*error="incompatible or empty LQG dimensions"; return false;
  }
  if(!K.allFinite()||!estimated_state.allFinite()) {
    if(error)*error="LQG input contains non-finite values"; return false;
  }
  Eigen::VectorXd candidate=-K*estimated_state;
  if(!candidate.allFinite()){if(error)*error="LQG control is non-finite";return false;}
  control=std::move(candidate);if(error)error->clear();return true;
}
} // namespace mca::control
