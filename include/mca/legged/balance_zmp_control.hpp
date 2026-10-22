#pragma once
#include <cmath>
#include <string>
#include <utility>
#include "mca/geometry/se3_so3.hpp"
namespace mca::legged {
struct ZmpInput {
  geometry::Vec3 center_of_mass_world{};
  geometry::Vec3 center_of_mass_acceleration_world{};
  double ground_height{0.0};
  double gravity_magnitude{9.81};
};
struct ZmpOutput { geometry::Vec3 position_world{}; bool inside_support{false}; };
struct RectangularSupportRegion {
  double min_x{-0.1}, max_x{0.1}, min_y{-0.1}, max_y{0.1};
};
inline bool estimate_zmp(const ZmpInput& input, const RectangularSupportRegion& support,
                         ZmpOutput& output, std::string* error = nullptr) {
  const auto fail=[error](const char* m){if(error)*error=m;return false;};
  const auto finite=[](geometry::Vec3 v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);};
  if(!finite(input.center_of_mass_world)||!finite(input.center_of_mass_acceleration_world)||
     !std::isfinite(input.ground_height)||!std::isfinite(input.gravity_magnitude)||
     input.gravity_magnitude<=0.0) return fail("COM, acceleration and gravity must be finite; gravity must be positive");
  if(!std::isfinite(support.min_x)||!std::isfinite(support.max_x)||
     !std::isfinite(support.min_y)||!std::isfinite(support.max_y)||
     support.min_x>support.max_x||support.min_y>support.max_y)
    return fail("support-region bounds must be finite and ordered");
  const double height=input.center_of_mass_world.z-input.ground_height;
  if(height<0.0) return fail("COM must not be below the support plane");
  ZmpOutput candidate;
  candidate.position_world={input.center_of_mass_world.x-height*input.center_of_mass_acceleration_world.x/input.gravity_magnitude,
    input.center_of_mass_world.y-height*input.center_of_mass_acceleration_world.y/input.gravity_magnitude,input.ground_height};
  if(!finite(candidate.position_world)) return fail("computed ZMP is non-finite");
  candidate.inside_support=candidate.position_world.x>=support.min_x&&candidate.position_world.x<=support.max_x&&
    candidate.position_world.y>=support.min_y&&candidate.position_world.y<=support.max_y;
  output=std::move(candidate); return true;
}
struct BalanceControllerInput {
  geometry::Vec3 center_of_mass_position_world{};
  geometry::Vec3 center_of_mass_velocity_world{};
  geometry::Vec3 desired_center_of_mass_position_world{};
  geometry::Vec3 desired_center_of_mass_velocity_world{};
  geometry::Vec3 proportional_gain{2.0,2.0,2.0};
  geometry::Vec3 derivative_gain{1.0,1.0,1.0};
  geometry::Vec3 maximum_acceleration{3.0,3.0,3.0};
};
struct BalanceControllerOutput { geometry::Vec3 desired_acceleration_world{}; };
inline bool compute_balance_acceleration(const BalanceControllerInput& input,
                                         BalanceControllerOutput& output,
                                         std::string* error=nullptr) {
  const auto fail=[error](const char* m){if(error)*error=m;return false;};
  const auto finite=[](geometry::Vec3 v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);};
  if(!finite(input.center_of_mass_position_world)||!finite(input.center_of_mass_velocity_world)||
     !finite(input.desired_center_of_mass_position_world)||!finite(input.desired_center_of_mass_velocity_world)||
     !finite(input.proportional_gain)||!finite(input.derivative_gain)||!finite(input.maximum_acceleration))
    return fail("balance controller vectors must be finite");
  if(input.proportional_gain.x<0||input.proportional_gain.y<0||input.proportional_gain.z<0||
     input.derivative_gain.x<0||input.derivative_gain.y<0||input.derivative_gain.z<0||
     input.maximum_acceleration.x<=0||input.maximum_acceleration.y<=0||input.maximum_acceleration.z<=0)
    return fail("gains must be non-negative and acceleration limits positive");
  const geometry::Vec3 ep=input.desired_center_of_mass_position_world-input.center_of_mass_position_world;
  const geometry::Vec3 ev=input.desired_center_of_mass_velocity_world-input.center_of_mass_velocity_world;
  BalanceControllerOutput candidate;
  candidate.desired_acceleration_world={
    std::clamp(input.proportional_gain.x*ep.x+input.derivative_gain.x*ev.x,-input.maximum_acceleration.x,input.maximum_acceleration.x),
    std::clamp(input.proportional_gain.y*ep.y+input.derivative_gain.y*ev.y,-input.maximum_acceleration.y,input.maximum_acceleration.y),
    std::clamp(input.proportional_gain.z*ep.z+input.derivative_gain.z*ev.z,-input.maximum_acceleration.z,input.maximum_acceleration.z)};
  if(!finite(candidate.desired_acceleration_world)) return fail("computed balance acceleration is non-finite");
  output=std::move(candidate);return true;
}
} // namespace mca::legged
