#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "mca/geometry/se3_so3.hpp"

namespace mca::uav {

struct EulerZYX {
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

class AttitudeRepresentation {
 public:
  // ZYX convention: yaw about world Z, pitch about intermediate Y, roll about body X.
  // Angles are radians; quaternions use (w,x,y,z) and map body coordinates to world.
  static bool from_euler_zyx(const EulerZYX& euler, geometry::Quaternion& output,
                             std::string* error = nullptr) {
    if (!std::isfinite(euler.roll) || !std::isfinite(euler.pitch) ||
        !std::isfinite(euler.yaw))
      return fail("Euler angles must be finite", error);
    const double cr = std::cos(euler.roll * 0.5);
    const double sr = std::sin(euler.roll * 0.5);
    const double cp = std::cos(euler.pitch * 0.5);
    const double sp = std::sin(euler.pitch * 0.5);
    const double cy = std::cos(euler.yaw * 0.5);
    const double sy = std::sin(euler.yaw * 0.5);
    geometry::Quaternion candidate{
      cr * cp * cy + sr * sp * sy,
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy
    };
    if (!geometry::normalize(candidate)) return fail("Euler conversion failed", error);
    output = candidate;
    return true;
  }

  static bool to_euler_zyx(geometry::Quaternion input, EulerZYX& output,
                           std::string* error = nullptr) {
    if (!finite(input) || !geometry::normalize(input))
      return fail("quaternion must be finite and have non-zero norm", error);
    const double sin_pitch = std::clamp(2.0 * (input.w * input.y - input.z * input.x),
                                        -1.0, 1.0);
    EulerZYX candidate;
    candidate.roll = std::atan2(2.0 * (input.w * input.x + input.y * input.z),
                                1.0 - 2.0 * (input.x * input.x + input.y * input.y));
    candidate.pitch = std::asin(sin_pitch);
    candidate.yaw = std::atan2(2.0 * (input.w * input.z + input.x * input.y),
                               1.0 - 2.0 * (input.y * input.y + input.z * input.z));
    // At gimbal lock roll/yaw are not individually observable; this is the
    // deterministic Euler representative returned by the standard ZYX formula.
    output = candidate;
    return true;
  }

  static bool from_rotation_matrix(const geometry::Mat3& matrix,
                                   geometry::Quaternion& output,
                                   std::string* error = nullptr) {
    for (double value : matrix.v)
      if (!std::isfinite(value)) return fail("rotation matrix must be finite", error);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        double product = 0.0;
        for (int k = 0; k < 3; ++k) product += matrix(k, r) * matrix(k, c);
        const double expected = r == c ? 1.0 : 0.0;
        if (std::abs(product - expected) > 1e-8)
          return fail("rotation matrix must be orthonormal", error);
      }
    }
    const double determinant =
        matrix(0,0) * (matrix(1,1)*matrix(2,2)-matrix(1,2)*matrix(2,1))
      - matrix(0,1) * (matrix(1,0)*matrix(2,2)-matrix(1,2)*matrix(2,0))
      + matrix(0,2) * (matrix(1,0)*matrix(2,1)-matrix(1,1)*matrix(2,0));
    if (std::abs(determinant - 1.0) > 1e-8)
      return fail("rotation matrix determinant must be +1", error);
    const geometry::Quaternion candidate = geometry::from_rotation_matrix(matrix);
    if (!finite(candidate)) return fail("rotation matrix conversion failed", error);
    output = candidate;
    return true;
  }

  static geometry::Mat3 to_rotation_matrix(geometry::Quaternion input) {
    return geometry::to_rotation_matrix(input);
  }

 private:
  static bool finite(geometry::Quaternion q) {
    return std::isfinite(q.w) && std::isfinite(q.x) &&
           std::isfinite(q.y) && std::isfinite(q.z);
  }
  static bool fail(const char* message, std::string* error) {
    if (error) *error = message;
    return false;
  }
};

}  // namespace mca::uav
