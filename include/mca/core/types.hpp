#pragma once

#include <cstdint>
#include <limits>
#include <string_view>

namespace mca::core {

struct Time {
  double seconds{0.0};
};

struct Duration {
  double seconds{0.0};
};

struct ScalarState {
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

enum class StatusCode : std::uint8_t {
  Ok = 0,
  InvalidArgument,
  NotConfigured,
  NumericalError,
  ConstraintViolation,
  RuntimeError
};

struct Status {
  StatusCode code{StatusCode::Ok};
  constexpr bool ok() const noexcept { return code == StatusCode::Ok; }
};

constexpr double kEpsilon = 1e-12;

constexpr double deg_to_rad(double degrees) noexcept {
  return degrees * 0.017453292519943295769236907684886;
}

constexpr double rad_to_deg(double radians) noexcept {
  return radians * 57.295779513082320876798154814105;
}

constexpr bool is_finite(double value) noexcept {
  return value == value && value != std::numeric_limits<double>::infinity() &&
         value != -std::numeric_limits<double>::infinity();
}

}  // namespace mca::core
