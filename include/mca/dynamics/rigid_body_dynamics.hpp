#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "mca/kinematics/forward_kinematics.hpp"

namespace mca::dynamics {

// Inertia tensor is expressed in the link frame, about the link-frame origin
// only when center_of_mass is zero; otherwise inertia_at_com is about the COM.
// All quantities use SI units.
struct LinkInertial {
  double mass{0.0};
  geometry::Vec3 center_of_mass{};
  geometry::Mat3 inertia_at_com{{0, 0, 0, 0, 0, 0, 0, 0, 0}};
};

class RigidBodyDynamics {
 public:
  // One inertial record per model link, in RobotModel::links order (including
  // the base link). The base is normally massless for a fixed-base arm.
  // Gravity is an acceleration vector in the base frame, e.g. (0,0,-9.81).
  static bool mass_matrix(const kinematics::RobotModel& model,
                          const std::vector<LinkInertial>& inertias,
                          const Eigen::VectorXd& q, Eigen::MatrixXd& out,
                          std::string* error = nullptr) {
    Eigen::MatrixXd candidate;
    if (!compute_mass(model, inertias, q, candidate, error)) return false;
    out = std::move(candidate);
    return true;
  }

  // Generalized Coriolis/centrifugal vector c(q,dq) = C(q,dq)*dq.
  // Uses central finite differences of the mass matrix; this reference
  // implementation favors transparent validation over high-DOF performance.
  static bool coriolis(const kinematics::RobotModel& model,
                       const std::vector<LinkInertial>& inertias,
                       const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                       Eigen::VectorXd& out, std::string* error = nullptr) {
    if (!valid_state(model, inertias, q, error)) return false;
    const Eigen::Index n = q.size();
    if (dq.size() != n || !dq.allFinite()) return fail("velocity vector is invalid", error);
    std::vector<Eigen::MatrixXd> derivatives(static_cast<std::size_t>(n));
    constexpr double eps = 1e-5;
    for (Eigen::Index k = 0; k < n; ++k) {
      Eigen::VectorXd plus = q, minus = q;
      plus[k] += eps;
      minus[k] -= eps;
      Eigen::MatrixXd mp, mm;
      if (!compute_mass(model, inertias, plus, mp, error) ||
          !compute_mass(model, inertias, minus, mm, error)) return false;
      derivatives[static_cast<std::size_t>(k)] = (mp - mm) / (2.0 * eps);
    }
    Eigen::VectorXd candidate = Eigen::VectorXd::Zero(n);
    for (Eigen::Index i = 0; i < n; ++i) {
      double value = 0.0;
      for (Eigen::Index j = 0; j < n; ++j) {
        for (Eigen::Index k = 0; k < n; ++k) {
          const double gamma = 0.5 *
              (derivatives[static_cast<std::size_t>(k)](i, j) + derivatives[static_cast<std::size_t>(j)](i, k) - derivatives[static_cast<std::size_t>(i)](j, k));
          value += gamma * dq[j] * dq[k];
        }
      }
      candidate[i] = value;
    }
    if (!candidate.allFinite()) return fail("Coriolis computation is non-finite", error);
    out = std::move(candidate);
    return true;
  }

  static bool gravity(const kinematics::RobotModel& model,
                      const std::vector<LinkInertial>& inertias,
                      const Eigen::VectorXd& q, geometry::Vec3 gravity_acceleration,
                      Eigen::VectorXd& out, std::string* error = nullptr) {
    if (!valid_state(model, inertias, q, error)) return false;
    if (!finite(gravity_acceleration)) return fail("gravity vector is non-finite", error);
    Eigen::VectorXd candidate = Eigen::VectorXd::Zero(q.size());
    kinematics::ForwardKinematicsResult fk;
    if (!forward(model, q, fk, error)) return false;
    std::size_t qi_count = 0;
    for (const auto& joint : model.joints) if (joint.type != kinematics::JointType::Fixed) ++qi_count;
    (void)qi_count;
    for (std::size_t li = 1; li < model.links.size(); ++li) {
      const auto& inertia = inertias[li];
      if (inertia.mass == 0.0) continue;
      const geometry::Vec3 com = geometry::transform_point(fk.link_poses[li], inertia.center_of_mass);
      Eigen::MatrixXd jv;
      if (!point_jacobian(model, q, li, com, jv, error)) return false;
      const Eigen::Vector3d g(gravity_acceleration.x, gravity_acceleration.y, gravity_acceleration.z);
      candidate.noalias() -= inertia.mass * jv.transpose() * g;
    }
    if (!candidate.allFinite()) return fail("gravity computation is non-finite", error);
    out = std::move(candidate);
    return true;
  }

  static bool bias_forces(const kinematics::RobotModel& model,
                          const std::vector<LinkInertial>& inertias,
                          const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                          geometry::Vec3 gravity_acceleration, Eigen::VectorXd& out,
                          std::string* error = nullptr) {
    Eigen::VectorXd c, g;
    if (!coriolis(model, inertias, q, dq, c, error) ||
        !gravity(model, inertias, q, gravity_acceleration, g, error)) return false;
    out = c + g;
    return true;
  }

 private:
  static bool finite(geometry::Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  }
  static bool finite(geometry::Mat3 m) {
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        if (!std::isfinite(m(r, c))) return false;
    return true;
  }
  static bool fail(const char* message, std::string* error) {
    if (error) *error = message;
    return false;
  }
  static bool valid_state(const kinematics::RobotModel& model,
                          const std::vector<LinkInertial>& inertias,
                          const Eigen::VectorXd& q, std::string* error) {
    std::string model_error;
    if (!model.valid(&model_error)) return fail(model_error.c_str(), error);
    if (inertias.size() != model.links.size()) return fail("inertial record count must match model links", error);
    std::size_t n = 0;
    for (const auto& j : model.joints) if (j.type != kinematics::JointType::Fixed) ++n;
    if (q.size() != static_cast<Eigen::Index>(n) || !q.allFinite())
      return fail("joint position vector is invalid", error);
    for (const auto& p : inertias) {
      if (!std::isfinite(p.mass) || p.mass < 0.0 || !finite(p.center_of_mass) || !finite(p.inertia_at_com))
        return fail("link inertial parameters are invalid", error);
      if (p.inertia_at_com(0,0) < -1e-12 || p.inertia_at_com(1,1) < -1e-12 ||
          p.inertia_at_com(2,2) < -1e-12)
        return fail("inertia tensor diagonal must be non-negative", error);
    }
    return true;
  }
  static bool forward(const kinematics::RobotModel& model, const Eigen::VectorXd& q,
                      kinematics::ForwardKinematicsResult& fk, std::string* error) {
    std::vector<double> values(static_cast<std::size_t>(q.size()));
    for (Eigen::Index i = 0; i < q.size(); ++i) values[static_cast<std::size_t>(i)] = q[i];
    if (!kinematics::ForwardKinematics::solve(model, values, fk, error)) return false;
    return true;
  }
  static bool point_jacobian(const kinematics::RobotModel& model, const Eigen::VectorXd& q,
                             std::size_t link_index, geometry::Vec3 point,
                             Eigen::MatrixXd& jv, std::string* error) {
    kinematics::ForwardKinematicsResult fk;
    if (!forward(model, q, fk, error)) return false;
    const Eigen::Index n = q.size();
    jv = Eigen::MatrixXd::Zero(3, n);
    Eigen::Index qi = 0;
    for (std::size_t ji = 0; ji < model.joints.size() && ji < link_index; ++ji) {
      const auto& joint = model.joints[ji];
      if (joint.type == kinematics::JointType::Fixed) continue;
      geometry::Pose3D frame;
      geometry::Vec3 axis;
      if (joint.standard_dh) {
        frame = fk.link_poses[ji];
        axis = geometry::rotate(frame.rotation, {0.0, 0.0, 1.0});
      } else {
        frame = geometry::compose(fk.link_poses[ji], joint.origin);
        axis = geometry::rotate(frame.rotation, joint.axis);
      }
      const double norm = geometry::norm(axis);
      if (!std::isfinite(norm) || norm <= 1e-12) return fail("joint axis is invalid", error);
      axis = (1.0 / norm) * axis;
      geometry::Vec3 linear{};
      if (joint.type == kinematics::JointType::Revolute ||
          joint.type == kinematics::JointType::Continuous) {
        linear = geometry::cross(axis, point - frame.position);
      } else if (joint.type == kinematics::JointType::Prismatic) {
        linear = axis;
      }
      jv(0, qi) = linear.x; jv(1, qi) = linear.y; jv(2, qi) = linear.z;
      ++qi;
    }
    return true;
  }
  static bool compute_mass(const kinematics::RobotModel& model,
                           const std::vector<LinkInertial>& inertias,
                           const Eigen::VectorXd& q, Eigen::MatrixXd& mass,
                           std::string* error) {
    if (!valid_state(model, inertias, q, error)) return false;
    kinematics::ForwardKinematicsResult fk;
    if (!forward(model, q, fk, error)) return false;
    const Eigen::Index n = q.size();
    Eigen::MatrixXd candidate = Eigen::MatrixXd::Zero(n, n);
    for (std::size_t li = 1; li < model.links.size(); ++li) {
      const auto& inertia = inertias[li];
      if (inertia.mass == 0.0) continue;
      const auto& pose = fk.link_poses[li];
      const geometry::Vec3 com = geometry::transform_point(pose, inertia.center_of_mass);
      Eigen::MatrixXd jv;
      if (!point_jacobian(model, q, li, com, jv, error)) return false;
      candidate.noalias() += inertia.mass * jv.transpose() * jv;
      Eigen::MatrixXd jw = Eigen::MatrixXd::Zero(3, n);
      Eigen::Index qi = 0;
      for (std::size_t ji = 0; ji < model.joints.size() && ji < li; ++ji) {
        const auto& joint = model.joints[ji];
        if (joint.type == kinematics::JointType::Fixed) continue;
        if (joint.type == kinematics::JointType::Revolute ||
            joint.type == kinematics::JointType::Continuous) {
          geometry::Vec3 axis;
          if (joint.standard_dh) axis = geometry::rotate(fk.link_poses[ji].rotation, {0.0,0.0,1.0});
          else axis = geometry::rotate(geometry::compose(fk.link_poses[ji], joint.origin).rotation, joint.axis);
          const double norm = geometry::norm(axis);
          if (norm <= 1e-12 || !std::isfinite(norm)) return fail("joint axis is invalid", error);
          axis = (1.0 / norm) * axis;
          jw(0,qi)=axis.x; jw(1,qi)=axis.y; jw(2,qi)=axis.z;
        }
        ++qi;
      }
      const geometry::Mat3 rotation_geometry = geometry::to_rotation_matrix(pose.rotation);
      Eigen::Matrix3d rotation;
      for (int r=0;r<3;++r) for (int c=0;c<3;++c) rotation(r,c)=rotation_geometry(r,c);
      Eigen::Matrix3d ic;
      for (int r=0;r<3;++r) for (int c=0;c<3;++c) ic(r,c)=inertia.inertia_at_com(r,c);
      const Eigen::Matrix3d iw = rotation * ic * rotation.transpose();
      candidate.noalias() += jw.transpose() * iw * jw;
    }
    candidate = 0.5 * (candidate + candidate.transpose()).eval();
    if (!candidate.allFinite()) return fail("mass matrix contains non-finite values", error);
    mass = std::move(candidate);
    return true;
  }
};

}  // namespace mca::dynamics
