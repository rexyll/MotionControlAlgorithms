#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "mca/geometry/se3_so3.hpp"

namespace mca::kinematics {

enum class JointType { Fixed, Revolute, Continuous, Prismatic };

enum class DhConvention { Standard };

struct JointLimits {
  double lower{-std::numeric_limits<double>::infinity()};
  double upper{std::numeric_limits<double>::infinity()};
  double effort{std::numeric_limits<double>::infinity()};
  double velocity{std::numeric_limits<double>::infinity()};
};

struct DhParameter {
  std::string name{};
  JointType type{JointType::Fixed};
  double a{0.0};
  double alpha{0.0};
  double d{0.0};
  double theta{0.0};
  JointLimits limits{};
};

struct RobotJoint {
  std::string name{};
  std::string parent_link{};
  std::string child_link{};
  JointType type{JointType::Fixed};
  geometry::Pose3D origin{};
  geometry::Vec3 axis{0.0, 0.0, 1.0};
  JointLimits limits{};
};

struct RobotModel {
  std::string name{};
  std::string root_link{};
  std::string tip_link{};
  std::vector<std::string> links{};
  std::vector<RobotJoint> joints{};
  DhConvention dh_convention{DhConvention::Standard};

  bool valid(std::string* error = nullptr) const {
    if (links.empty()) return fail("model has no links", error);
    if (joints.size() + 1U != links.size()) return fail("model is not a serial chain", error);
    if (root_link.empty() || tip_link.empty()) return fail("root/tip link is empty", error);
    if (std::find(links.begin(), links.end(), root_link) == links.end()) return fail("root link is missing", error);
    if (std::find(links.begin(), links.end(), tip_link) == links.end()) return fail("tip link is missing", error);
    std::string parent = root_link;
    for (const auto& joint : joints) {
      if (joint.parent_link != parent) return fail("joint order does not form a serial chain", error);
      if (joint.child_link.empty() || std::find(links.begin(), links.end(), joint.child_link) == links.end()) {
        return fail("joint child link is missing", error);
      }
      if (!finite_pose(joint.origin) || !finite(joint.axis.x) || !finite(joint.axis.y) || !finite(joint.axis.z)) {
        return fail("joint contains non-finite geometry", error);
      }
      const double axis_norm = geometry::norm(joint.axis);
      if (joint.type != JointType::Fixed && axis_norm <= 1e-12) return fail("movable joint has zero axis", error);
      parent = joint.child_link;
    }
    if (parent != tip_link) return fail("tip link is not the end of the chain", error);
    return true;
  }

 private:
  static bool finite(double value) { return std::isfinite(value); }
  static bool finite_pose(const geometry::Pose3D& pose) {
    return finite(pose.position.x) && finite(pose.position.y) && finite(pose.position.z) &&
           finite(pose.rotation.w) && finite(pose.rotation.x) && finite(pose.rotation.y) && finite(pose.rotation.z);
  }
  static bool fail(const char* message, std::string* error) {
    if (error != nullptr) *error = message;
    return false;
  }
};

class DhModelAdapter {
 public:
  static bool build(const std::string& name, const std::string& root_link,
                    const std::vector<DhParameter>& parameters, RobotModel& output,
                    std::string* error = nullptr) {
    if (root_link.empty()) return fail("root link is empty", error);
    if (parameters.empty()) return fail("DH parameter list is empty", error);

    RobotModel candidate;
    candidate.name = name;
    candidate.root_link = root_link;
    candidate.dh_convention = DhConvention::Standard;
    candidate.links.push_back(root_link);
    std::string parent = root_link;
    for (std::size_t i = 0; i < parameters.size(); ++i) {
      const auto& p = parameters[i];
      if (p.name.empty()) return fail("DH joint name is empty", error);
      if (!finite(p.a) || !finite(p.alpha) || !finite(p.d) || !finite(p.theta)) {
        return fail("DH parameter is non-finite", error);
      }
      if (!valid_limits(p.limits)) return fail("invalid DH joint limits", error);
      const std::string child = "link_" + std::to_string(i + 1U);
      RobotJoint joint;
      joint.name = p.name;
      joint.parent_link = parent;
      joint.child_link = child;
      joint.type = p.type;
      joint.origin = dh_zero_configuration(p);
      joint.axis = {0.0, 0.0, 1.0};
      joint.limits = p.limits;
      candidate.joints.push_back(joint);
      candidate.links.push_back(child);
      parent = child;
    }
    candidate.tip_link = parent;
    if (!candidate.valid(error)) return false;
    output = std::move(candidate);
    return true;
  }

  static geometry::Pose3D dh_zero_configuration(const DhParameter& p) {
    const double ca = std::cos(p.alpha);
    const double sa = std::sin(p.alpha);
    const double ct = std::cos(p.theta);
    const double st = std::sin(p.theta);
    geometry::Pose3D pose;
    pose.position = {p.a * ct, p.a * st, p.d};
    geometry::Mat3 r;
    r(0, 0) = ct; r(0, 1) = -st * ca; r(0, 2) = st * sa;
    r(1, 0) = st; r(1, 1) = ct * ca; r(1, 2) = -ct * sa;
    r(2, 0) = 0.0; r(2, 1) = sa; r(2, 2) = ca;
    pose.rotation = geometry::from_rotation_matrix(r);
    return pose;
  }

 private:
  static bool finite(double v) { return std::isfinite(v); }
  static bool valid_limits(const JointLimits& l) {
    return (std::isinf(l.lower) || finite(l.lower)) && (std::isinf(l.upper) || finite(l.upper)) &&
           (std::isinf(l.effort) || finite(l.effort)) && (std::isinf(l.velocity) || finite(l.velocity)) &&
           l.lower <= l.upper;
  }
  static bool fail(const char* message, std::string* error) {
    if (error != nullptr) *error = message;
    return false;
  }
};

class UrdfModelAdapter {
 public:
  // Dependency-free URDF subset parser for serial chains. Supported tags:
  // robot, link, joint, parent, child, origin(xyz/rpy), axis(xyz), limit.
  static bool parse(const std::string& urdf, RobotModel& output, std::string* error = nullptr) {
    RobotModel candidate;
    const auto robot_tag = first_tag(urdf, "robot");
    if (robot_tag.empty()) return fail("robot tag is missing", error);
    candidate.name = attribute(robot_tag, "name");
    if (candidate.name.empty()) return fail("robot name is missing", error);

    const auto link_tags = all_tags(urdf, "link");
    if (link_tags.empty()) return fail("URDF has no links", error);
    for (const auto& tag : link_tags) {
      const std::string name = attribute(tag, "name");
      if (name.empty()) return fail("link name is empty", error);
      if (std::find(candidate.links.begin(), candidate.links.end(), name) != candidate.links.end()) {
        return fail("duplicate link name", error);
      }
      candidate.links.push_back(name);
    }

    const auto joint_blocks = all_elements(urdf, "joint");
    if (joint_blocks.empty()) return fail("URDF has no joints", error);
    for (const auto& block : joint_blocks) {
      const std::string open = first_tag(block, "joint");
      RobotJoint joint;
      joint.name = attribute(open, "name");
      const std::string type = attribute(open, "type");
      if (joint.name.empty() || type.empty()) return fail("joint name/type is missing", error);
      if (!parse_joint_type(type, joint.type)) return fail("unsupported joint type", error);
      const std::string parent_tag = first_tag(block, "parent");
      const std::string child_tag = first_tag(block, "child");
      joint.parent_link = attribute(parent_tag, "link");
      joint.child_link = attribute(child_tag, "link");
      if (joint.parent_link.empty() || joint.child_link.empty()) return fail("joint parent/child is missing", error);
      if (!parse_origin(block, joint.origin, error)) return false;
      if (!parse_axis(block, joint.axis, joint.type, error)) return false;
      if (!parse_limit(block, joint.limits, joint.type, error)) return false;
      candidate.joints.push_back(joint);
    }

    std::vector<std::string> children;
    for (const auto& joint : candidate.joints) {
      if (std::find(candidate.links.begin(), candidate.links.end(), joint.parent_link) == candidate.links.end() ||
          std::find(candidate.links.begin(), candidate.links.end(), joint.child_link) == candidate.links.end()) {
        return fail("joint references an unknown link", error);
      }
      if (std::find(children.begin(), children.end(), joint.child_link) != children.end()) {
        return fail("link has multiple parent joints", error);
      }
      children.push_back(joint.child_link);
    }
    std::vector<std::string> roots;
    for (const auto& link : candidate.links) {
      if (std::find(children.begin(), children.end(), link) == children.end()) roots.push_back(link);
    }
    if (roots.size() != 1U) return fail("URDF is not a single-root serial tree", error);
    candidate.root_link = roots.front();

    std::vector<RobotJoint> ordered;
    ordered.reserve(candidate.joints.size());
    std::string current = candidate.root_link;
    while (ordered.size() < candidate.joints.size()) {
      auto it = std::find_if(candidate.joints.begin(), candidate.joints.end(), [&](const RobotJoint& j) {
        return j.parent_link == current;
      });
      if (it == candidate.joints.end()) return fail("URDF is not a single serial chain", error);
      ordered.push_back(*it);
      current = it->child_link;
    }
    candidate.joints = std::move(ordered);
    candidate.tip_link = current;
    if (!candidate.valid(error)) return false;
    output = std::move(candidate);
    return true;
  }

 private:
  static bool fail(const char* message, std::string* error) {
    if (error != nullptr) *error = message;
    return false;
  }

  static std::string first_tag(const std::string& text, const std::string& name) {
    const std::regex pattern("<" + name + R"(\b[^>]*>)");
    std::smatch match;
    if (!std::regex_search(text, match, pattern)) return {};
    return match.str(0);
  }

  static std::vector<std::string> all_tags(const std::string& text, const std::string& name) {
    std::vector<std::string> result;
    const std::regex pattern("<" + name + R"(\b[^>]*>)");
    for (std::sregex_iterator it(text.begin(), text.end(), pattern), end; it != end; ++it) result.push_back(it->str());
    return result;
  }

  static std::vector<std::string> all_elements(const std::string& text, const std::string& name) {
    std::vector<std::string> result;
    const std::regex pattern("<" + name + R"(\b[^>]*>.*?</)" + name + ">)", std::regex::ECMAScript | std::regex::icase);
    for (std::sregex_iterator it(text.begin(), text.end(), pattern), end; it != end; ++it) result.push_back(it->str());
    return result;
  }

  static std::string attribute(const std::string& tag, const std::string& name) {
    const std::regex pattern(name + R"(\s*=\s*[\"]([^\"]*)[\"])");
    std::smatch match;
    if (std::regex_search(tag, match, pattern)) return match[1].str();
    return {};
  }

  static bool parse_values(const std::string& text, const std::string& tag_name, const char* attr,
                           double* values, std::size_t count) {
    const std::string tag = first_tag(text, tag_name);
    if (tag.empty()) return false;
    std::istringstream stream(attribute(tag, attr));
    for (std::size_t i = 0; i < count; ++i) {
      if (!(stream >> values[i]) || !std::isfinite(values[i])) return false;
    }
    return true;
  }

  static bool parse_origin(const std::string& block, geometry::Pose3D& pose, std::string* error) {
    const std::string tag = first_tag(block, "origin");
    if (tag.empty()) return true;
    double xyz[3]{};
    double rpy[3]{};
    if (!parse_attribute_vector(tag, "xyz", xyz) || !parse_attribute_vector(tag, "rpy", rpy)) return fail("invalid origin", error);
    const double cr = std::cos(rpy[0] * 0.5); const double sr = std::sin(rpy[0] * 0.5);
    const double cp = std::cos(rpy[1] * 0.5); const double sp = std::sin(rpy[1] * 0.5);
    const double cy = std::cos(rpy[2] * 0.5); const double sy = std::sin(rpy[2] * 0.5);
    pose.position = {xyz[0], xyz[1], xyz[2]};
    pose.rotation = geometry::normalized({cr * cp * cy + sr * sp * sy,
                                           sr * cp * cy - cr * sp * sy,
                                           cr * sp * cy + sr * cp * sy,
                                           cr * cp * sy - sr * sp * cy});
    return true;
  }

  static bool parse_axis(const std::string& block, geometry::Vec3& axis, JointType type, std::string* error) {
    const std::string tag = first_tag(block, "axis");
    if (tag.empty()) {
      axis = {0.0, 0.0, 1.0};
      return type == JointType::Fixed || true;
    }
    double v[3]{};
    if (!parse_attribute_vector(tag, "xyz", v)) return fail("invalid joint axis", error);
    axis = {v[0], v[1], v[2]};
    const double n = geometry::norm(axis);
    if (type != JointType::Fixed && n <= 1e-12) return fail("movable joint has zero axis", error);
    if (n > 1e-12) axis = (1.0 / n) * axis;
    return true;
  }

  static bool parse_limit(const std::string& block, JointLimits& limits, JointType type, std::string* error) {
    const std::string tag = first_tag(block, "limit");
    if (tag.empty()) return true;
    auto read = [&](const char* name, double& target) {
      const std::string value = attribute(tag, name);
      if (value.empty()) return true;
      try {
        target = std::stod(value);
      } catch (...) {
        return false;
      }
      return std::isfinite(target);
    };
    if (type == JointType::Revolute || type == JointType::Prismatic) {
      if (!read("lower", limits.lower) || !read("upper", limits.upper)) return fail("invalid joint limits", error);
    }
    if (!read("effort", limits.effort) || !read("velocity", limits.velocity)) return fail("invalid joint limits", error);
    if (limits.lower > limits.upper) return fail("joint lower limit exceeds upper limit", error);
    return true;
  }

  static bool parse_attribute_vector(const std::string& tag, const char* name, double out[3]) {
    std::istringstream stream(attribute(tag, name));
    return static_cast<bool>(stream >> out[0] >> out[1] >> out[2]) &&
           std::isfinite(out[0]) && std::isfinite(out[1]) && std::isfinite(out[2]);
  }

  static bool parse_joint_type(const std::string& type, JointType& output) {
    if (type == "fixed") output = JointType::Fixed;
    else if (type == "revolute") output = JointType::Revolute;
    else if (type == "continuous") output = JointType::Continuous;
    else if (type == "prismatic") output = JointType::Prismatic;
    else return false;
    return true;
  }
};

}  // namespace mca::kinematics
