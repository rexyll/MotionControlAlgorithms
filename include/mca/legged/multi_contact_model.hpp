#pragma once

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "mca/legged/contact_model.hpp"

namespace mca::legged {

struct MultiContactPoint {
  geometry::Vec3 position_world{};
  ContactPointState state{};
  ContactModelParameters parameters{};
};

struct MultiContactInput {
  geometry::Vec3 reference_point_world{};
  std::vector<MultiContactPoint> points;
};

struct MultiContactOutput {
  std::vector<ContactWrench> contacts;
  geometry::Vec3 total_force_world{};
  geometry::Vec3 total_torque_world{};
  std::size_t active_contacts{0};
};

// Evaluates independent point contacts and aggregates their force and moment
// about reference_point_world. Output is committed only when every contact is
// valid and successfully evaluated.
class MultiContactModel {
 public:
  bool evaluate(const MultiContactInput& input,
                MultiContactOutput* output,
                std::string* error = nullptr) const {
    const auto fail = [error](const char* message) {
      if (error) *error = message;
      return false;
    };
    if (output == nullptr) return fail("output must not be null");
    if (!finite(input.reference_point_world))
      return fail("reference point must be finite");
    if (input.points.empty()) return fail("at least one contact point is required");

    MultiContactOutput candidate;
    candidate.contacts.reserve(input.points.size());
    for (const auto& point : input.points) {
      if (!finite(point.position_world))
        return fail("contact position must be finite");

      ContactWrench wrench;
      std::string contact_error;
      if (!evaluate_contact(point.parameters, point.state, wrench, &contact_error)) {
        if (error) *error = contact_error;
        return false;
      }

      candidate.contacts.push_back(wrench);
      candidate.total_force_world = candidate.total_force_world + wrench.force_world;
      const geometry::Vec3 lever_arm = point.position_world - input.reference_point_world;
      candidate.total_torque_world =
          candidate.total_torque_world + geometry::cross(lever_arm, wrench.force_world);
      if (wrench.active) ++candidate.active_contacts;
    }

    if (!finite(candidate.total_force_world) || !finite(candidate.total_torque_world))
      return fail("aggregated wrench is non-finite");
    *output = std::move(candidate);
    return true;
  }

 private:
  static bool finite(const geometry::Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
  }
};

}  // namespace mca::legged
