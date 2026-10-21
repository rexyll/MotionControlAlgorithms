#pragma once

#include "mca/trajectory/scalar.hpp"

namespace mca::trajectory {

// Cubic Hermite trajectory with position and velocity specified at both ends.
// Time is in seconds; positions use the caller's units.
class CubicTrajectory final : public ISampledScalarTrajectory {
 public:
  CubicTrajectory(double start_position, double start_velocity,
                  double finish_position, double finish_velocity,
                  double duration_s) noexcept
      : start_position_(start_position), start_velocity_(start_velocity),
        finish_position_(finish_position), finish_velocity_(finish_velocity),
        duration_s_(duration_s) {
    if (duration_s_ > 0.0) {
      const double t2 = duration_s_ * duration_s_;
      const double t3 = t2 * duration_s_;
      a2_ = (3.0 * (finish_position_ - start_position_) -
             (2.0 * start_velocity_ + finish_velocity_) * duration_s_) / t2;
      a3_ = (2.0 * (start_position_ - finish_position_) +
             (start_velocity_ + finish_velocity_) * duration_s_) / t3;
    }
  }

  ScalarSample sample(double time_s) const noexcept override {
    if (!(duration_s_ > 0.0)) return {finish_position_, 0.0, 0.0};
    if (time_s <= 0.0) return {start_position_, start_velocity_, 2.0 * a2_};
    if (time_s >= duration_s_) {
      return {finish_position_, finish_velocity_,
              2.0 * a2_ + 6.0 * a3_ * duration_s_};
    }
    const double t2 = time_s * time_s;
    const double t3 = t2 * time_s;
    return {start_position_ + start_velocity_ * time_s + a2_ * t2 + a3_ * t3,
            start_velocity_ + 2.0 * a2_ * time_s + 3.0 * a3_ * t2,
            2.0 * a2_ + 6.0 * a3_ * time_s};
  }

 private:
  double start_position_{0.0};
  double start_velocity_{0.0};
  double finish_position_{0.0};
  double finish_velocity_{0.0};
  double duration_s_{0.0};
  double a2_{0.0};
  double a3_{0.0};
};

// Quintic trajectory matching position, velocity and acceleration at both ends.
class QuinticTrajectory final : public ISampledScalarTrajectory {
 public:
  QuinticTrajectory(double start_position, double start_velocity,
                    double start_acceleration, double finish_position,
                    double finish_velocity, double finish_acceleration,
                    double duration_s) noexcept
      : start_position_(start_position), start_velocity_(start_velocity),
        start_acceleration_(start_acceleration), finish_position_(finish_position),
        finish_velocity_(finish_velocity), finish_acceleration_(finish_acceleration),
        duration_s_(duration_s) {
    if (duration_s_ > 0.0) {
      const double t = duration_s_;
      const double t2 = t * t;
      const double t3 = t2 * t;
      const double t4 = t3 * t;
      const double t5 = t4 * t;
      const double dp = finish_position_ - start_position_ - start_velocity_ * t -
                        0.5 * start_acceleration_ * t2;
      const double dv = finish_velocity_ - start_velocity_ - start_acceleration_ * t;
      const double da = finish_acceleration_ - start_acceleration_;
      a3_ = (10.0 * dp - 4.0 * dv * t + 0.5 * da * t2) / t3;
      a4_ = (-15.0 * dp + 7.0 * dv * t - da * t2) / t4;
      a5_ = (6.0 * dp - 3.0 * dv * t + 0.5 * da * t2) / t5;
    }
  }

  ScalarSample sample(double time_s) const noexcept override {
    if (!(duration_s_ > 0.0)) return {finish_position_, 0.0, 0.0};
    if (time_s <= 0.0) {
      return {start_position_, start_velocity_, start_acceleration_};
    }
    if (time_s >= duration_s_) {
      return {finish_position_, finish_velocity_, finish_acceleration_};
    }
    const double t2 = time_s * time_s;
    const double t3 = t2 * time_s;
    const double t4 = t3 * time_s;
    const double t5 = t4 * time_s;
    return {start_position_ + start_velocity_ * time_s +
                0.5 * start_acceleration_ * t2 + a3_ * t3 + a4_ * t4 + a5_ * t5,
            start_velocity_ + start_acceleration_ * time_s +
                3.0 * a3_ * t2 + 4.0 * a4_ * t3 + 5.0 * a5_ * t4,
            start_acceleration_ + 6.0 * a3_ * time_s +
                12.0 * a4_ * t2 + 20.0 * a5_ * t3};
  }

 private:
  double start_position_{0.0};
  double start_velocity_{0.0};
  double start_acceleration_{0.0};
  double finish_position_{0.0};
  double finish_velocity_{0.0};
  double finish_acceleration_{0.0};
  double duration_s_{0.0};
  double a3_{0.0};
  double a4_{0.0};
  double a5_{0.0};
};

}  // namespace mca::trajectory
