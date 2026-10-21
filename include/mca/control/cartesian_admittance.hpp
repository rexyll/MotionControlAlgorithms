#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "mca/geometry/se3_so3.hpp"

namespace mca::control {

// Stateful Cartesian admittance controller.
// M*a = W_ext + W_ff + K*(X_d-X) - D*(V-V_d)
// with base-frame [vx,vy,vz,wx,wy,wz] ordering.
// The commanded pose/twist are integrated with explicit Euler translation and
// exponential-map rotation. Limits are software command clamps, not hardware safety.
class CartesianAdmittance {
 public:
  CartesianAdmittance() = default;

  bool configure(const Eigen::VectorXd& virtual_mass,
                 const Eigen::VectorXd& damping,
                 const Eigen::VectorXd& stiffness,
                 double dt,
                 const Eigen::VectorXd& acceleration_limits = Eigen::VectorXd(),
                 const Eigen::VectorXd& velocity_limits = Eigen::VectorXd(),
                 const Eigen::VectorXd& wrench_limits = Eigen::VectorXd(),
                 std::string* error = nullptr) {
    if (!valid_vector(virtual_mass) || virtual_mass.size() != 6 ||
        (virtual_mass.array() <= 0.0).any())
      return fail("Cartesian-admittance virtual mass must contain six positive finite values", error);
    if (!valid_nonnegative(damping, 6))
      return fail("Cartesian-admittance damping must contain six non-negative finite values", error);
    if (!valid_nonnegative(stiffness, 6))
      return fail("Cartesian-admittance stiffness must contain six non-negative finite values", error);
    if (!std::isfinite(dt) || dt <= 0.0)
      return fail("Cartesian-admittance dt must be positive and finite", error);
    if (!valid_limits(acceleration_limits, "acceleration", error) ||
        !valid_limits(velocity_limits, "velocity", error) ||
        !valid_limits(wrench_limits, "wrench", error))
      return false;

    virtual_mass_ = virtual_mass;
    damping_ = damping;
    stiffness_ = stiffness;
    dt_ = dt;
    acceleration_limits_ = acceleration_limits;
    velocity_limits_ = velocity_limits;
    wrench_limits_ = wrench_limits;
    configured_ = true;
    initialized_ = false;
    return true;
  }

  bool reset(const geometry::Pose3D& pose,
             const Eigen::VectorXd& twist = Eigen::VectorXd(),
             std::string* error = nullptr) {
    if (!configured_)
      return fail("Cartesian-admittance controller is not configured", error);
    if (!pose_finite(pose))
      return fail("Cartesian-admittance reset pose must be finite", error);
    Eigen::VectorXd candidate = twist;
    if (candidate.size() == 0) candidate = Eigen::VectorXd::Zero(6);
    if (candidate.size() != 6 || !candidate.allFinite())
      return fail("Cartesian-admittance reset twist must contain six finite values", error);
    commanded_pose_ = pose;
    commanded_pose_.rotation = geometry::normalized(commanded_pose_.rotation);
    if (!pose_finite(commanded_pose_))
      return fail("Cartesian-admittance reset pose rotation is invalid", error);
    commanded_twist_ = candidate;
    last_acceleration_ = Eigen::VectorXd::Zero(6);
    initialized_ = true;
    return true;
  }

  bool update(const geometry::Pose3D& desired_pose,
              const Eigen::VectorXd& desired_twist,
              const Eigen::VectorXd& external_wrench,
              const Eigen::VectorXd& feedforward_wrench,
              geometry::Pose3D& out_pose,
              Eigen::VectorXd& out_twist,
              Eigen::VectorXd& out_acceleration,
              std::string* error = nullptr) {
    if (!configured_ || !initialized_)
      return fail("Cartesian-admittance controller must be configured and reset before update", error);
    if (!pose_finite(desired_pose))
      return fail("Cartesian-admittance desired pose must be finite", error);
    if (!valid_six(desired_twist) || !valid_six(external_wrench) || !valid_six(feedforward_wrench))
      return fail("Cartesian-admittance update vectors must contain six finite values", error);

    Eigen::VectorXd wrench = external_wrench + feedforward_wrench;
    if (wrench_limits_.size() != 0) clamp_symmetric(wrench, wrench_limits_);

    const geometry::Vec3 position_error = desired_pose.position - commanded_pose_.position;
    // R_desired * R_current^T is a base-frame orientation error. Its logarithm
    // is already expressed in the base frame, so do not rotate it again.
    const geometry::Quaternion q_error = geometry::normalized(
        geometry::multiply(desired_pose.rotation, geometry::conjugate(commanded_pose_.rotation)));
    const geometry::Vec3 orientation_error_base = geometry::log_so3(q_error);

    Eigen::VectorXd pose_error(6);
    pose_error << position_error.x, position_error.y, position_error.z,
                  orientation_error_base.x, orientation_error_base.y,
                  orientation_error_base.z;

    Eigen::VectorXd acceleration =
        (wrench + stiffness_.cwiseProduct(pose_error) -
         damping_.cwiseProduct(commanded_twist_ - desired_twist)).cwiseQuotient(virtual_mass_);
    if (acceleration_limits_.size() != 0) clamp_symmetric(acceleration, acceleration_limits_);

    Eigen::VectorXd candidate_twist = commanded_twist_ + dt_ * acceleration;
    if (velocity_limits_.size() != 0) clamp_symmetric(candidate_twist, velocity_limits_);

    geometry::Pose3D candidate_pose = commanded_pose_;
    candidate_pose.position.x += dt_ * candidate_twist[0];
    candidate_pose.position.y += dt_ * candidate_twist[1];
    candidate_pose.position.z += dt_ * candidate_twist[2];
    const geometry::Vec3 angular_step{dt_ * candidate_twist[3], dt_ * candidate_twist[4], dt_ * candidate_twist[5]};
    candidate_pose.rotation = geometry::normalized(
        geometry::multiply(geometry::exp_so3(angular_step), commanded_pose_.rotation));

    if (!pose_finite(candidate_pose) || !candidate_twist.allFinite() || !acceleration.allFinite())
      return fail("Cartesian-admittance update produced non-finite state", error);

    commanded_pose_ = candidate_pose;
    commanded_twist_ = std::move(candidate_twist);
    last_acceleration_ = std::move(acceleration);
    out_pose = commanded_pose_;
    out_twist = commanded_twist_;
    out_acceleration = last_acceleration_;
    return true;
  }

  bool configured() const noexcept { return configured_; }
  bool initialized() const noexcept { return initialized_; }
  const geometry::Pose3D& commanded_pose() const noexcept { return commanded_pose_; }
  const Eigen::VectorXd& commanded_twist() const noexcept { return commanded_twist_; }
  const Eigen::VectorXd& last_acceleration() const noexcept { return last_acceleration_; }

 private:
  static bool valid_vector(const Eigen::VectorXd& value) {
    return value.size() == 6 && value.allFinite();
  }
  static bool valid_six(const Eigen::VectorXd& value) { return valid_vector(value); }
  static bool valid_nonnegative(const Eigen::VectorXd& value, Eigen::Index size) {
    return value.size() == size && value.allFinite() && (value.array() >= 0.0).all();
  }
  static bool valid_limits(const Eigen::VectorXd& value, const char* name, std::string* error) {
    if (value.size() == 0) return true;
    if (!valid_six(value) || (value.array() < 0.0).any()) {
      if (error) *error = std::string("Cartesian-admittance ") + name + " limits must be six finite non-negative values";
      return false;
    }
    return true;
  }
  static void clamp_symmetric(Eigen::VectorXd& value, const Eigen::VectorXd& limits) {
    for (Eigen::Index i = 0; i < 6; ++i)
      value[i] = std::max(-limits[i], std::min(limits[i], value[i]));
  }
  static bool pose_finite(const geometry::Pose3D& pose) {
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(pose.position.z) || !std::isfinite(pose.rotation.w) ||
        !std::isfinite(pose.rotation.x) || !std::isfinite(pose.rotation.y) ||
        !std::isfinite(pose.rotation.z))
      return false;
    const double rotation_norm = geometry::quaternion_norm(pose.rotation);
    return std::isfinite(rotation_norm) && rotation_norm > 1e-12;
  }
  static bool fail(const char* message, std::string* error) {
    if (error) *error = message;
    return false;
  }

  Eigen::VectorXd virtual_mass_;
  Eigen::VectorXd damping_;
  Eigen::VectorXd stiffness_;
  Eigen::VectorXd acceleration_limits_;
  Eigen::VectorXd velocity_limits_;
  Eigen::VectorXd wrench_limits_;
  Eigen::VectorXd commanded_twist_;
  Eigen::VectorXd last_acceleration_;
  geometry::Pose3D commanded_pose_{};
  double dt_{0.0};
  bool configured_{false};
  bool initialized_{false};
};

}  // namespace mca::control
