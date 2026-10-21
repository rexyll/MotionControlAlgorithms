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
struct JointLimits { double lower{-std::numeric_limits<double>::infinity()}; double upper{std::numeric_limits<double>::infinity()}; double effort{std::numeric_limits<double>::infinity()}; double velocity{std::numeric_limits<double>::infinity()}; };
struct DhParameter { std::string name{}; JointType type{JointType::Fixed}; double a{0}; double alpha{0}; double d{0}; double theta{0}; JointLimits limits{}; };
struct RobotJoint { std::string name{}; std::string parent_link{}; std::string child_link{}; JointType type{JointType::Fixed}; geometry::Pose3D origin{}; geometry::Vec3 axis{0,0,1}; JointLimits limits{}; };

struct RobotModel {
  std::string name{}; std::string root_link{}; std::string tip_link{}; std::vector<std::string> links{}; std::vector<RobotJoint> joints{}; DhConvention dh_convention{DhConvention::Standard};
  bool valid(std::string* error=nullptr) const {
    if (links.empty()) return fail("model has no links",error);
    if (joints.size()+1U!=links.size()) return fail("model is not a serial chain",error);
    if (root_link.empty()||tip_link.empty()) return fail("root/tip link is empty",error);
    if (!contains(links,root_link)||!contains(links,tip_link)) return fail("root/tip link is missing",error);
    std::string parent=root_link;
    for(const auto& j:joints){
      if(j.parent_link!=parent) return fail("joint order does not form a serial chain",error);
      if(j.child_link.empty()||!contains(links,j.child_link)) return fail("joint child link is missing",error);
      if(!finite_pose(j.origin)||!std::isfinite(j.axis.x)||!std::isfinite(j.axis.y)||!std::isfinite(j.axis.z)) return fail("joint contains non-finite geometry",error);
      if(j.type!=JointType::Fixed&&geometry::norm(j.axis)<=1e-12) return fail("movable joint has zero axis",error);
      parent=j.child_link;
    }
    return parent==tip_link ? true : fail("tip link is not the end of the chain",error);
  }
 private:
  static bool contains(const std::vector<std::string>& v,const std::string& x){return std::find(v.begin(),v.end(),x)!=v.end();}
  static bool finite_pose(const geometry::Pose3D& p){return std::isfinite(p.position.x)&&std::isfinite(p.position.y)&&std::isfinite(p.position.z)&&std::isfinite(p.rotation.w)&&std::isfinite(p.rotation.x)&&std::isfinite(p.rotation.y)&&std::isfinite(p.rotation.z);}
  static bool fail(const char* m,std::string* e){if(e)*e=m;return false;}
};

class DhModelAdapter {
 public:
  static bool build(const std::string& name,const std::string& root,const std::vector<DhParameter>& ps,RobotModel& out,std::string* error=nullptr){
    if(root.empty()) return fail("root link is empty",error); if(ps.empty()) return fail("DH parameter list is empty",error);
    RobotModel c; c.name=name;c.root_link=root;c.links.push_back(root);std::string parent=root;
    for(std::size_t i=0;i<ps.size();++i){const auto& p=ps[i];if(p.name.empty())return fail("DH joint name is empty",error);if(!finite(p.a)||!finite(p.alpha)||!finite(p.d)||!finite(p.theta))return fail("DH parameter is non-finite",error);if(!limits_valid(p.limits))return fail("invalid DH joint limits",error);RobotJoint j;j.name=p.name;j.parent_link=parent;j.child_link="link_"+std::to_string(i+1U);j.type=p.type;j.origin=dh_zero_configuration(p);j.limits=p.limits;c.joints.push_back(j);c.links.push_back(j.child_link);parent=j.child_link;}
    c.tip_link=parent;if(!c.valid(error))return false;out=std::move(c);return true;
  }
  static geometry::Pose3D dh_zero_configuration(const DhParameter& p){const double ca=std::cos(p.alpha),sa=std::sin(p.alpha),ct=std::cos(p.theta),st=std::sin(p.theta);geometry::Pose3D pose;pose.position={p.a*ct,p.a*st,p.d};geometry::Mat3 r;r(0,0)=ct;r(0,1)=-st*ca;r(0,2)=st*sa;r(1,0)=st;r(1,1)=ct*ca;r(1,2)=-ct*sa;r(2,0)=0;r(2,1)=sa;r(2,2)=ca;pose.rotation=geometry::from_rotation_matrix(r);return pose;}
 private:
  static bool finite(double x){return std::isfinite(x);} static bool limits_valid(const JointLimits& l){return (std::isinf(l.lower)||finite(l.lower))&&(std::isinf(l.upper)||finite(l.upper))&&(std::isinf(l.effort)||finite(l.effort))&&(std::isinf(l.velocity)||finite(l.velocity))&&l.lower<=l.upper;} static bool fail(const char* m,std::string* e){if(e)*e=m;return false;}
};

class UrdfModelAdapter {
 public:
  static bool parse(const std::string& urdf,RobotModel& out,std::string* error=nullptr){
    RobotModel c;const std::string rt=first_tag(urdf,"robot");if(rt.empty())return fail("robot tag is missing",error);c.name=attribute(rt,"name");if(c.name.empty())return fail("robot name is missing",error);
    for(const auto& tag:all_tags(urdf,"link")){const std::string n=attribute(tag,"name");if(n.empty())return fail("link name is empty",error);if(contains(c.links,n))return fail("duplicate link name",error);c.links.push_back(n);}if(c.links.empty())return fail("URDF has no links",error);
    const auto blocks=all_elements(urdf,"joint");if(blocks.empty())return fail("URDF has no joints",error);
    for(const auto& b:blocks){const std::string open=first_tag(b,"joint");RobotJoint j;j.name=attribute(open,"name");if(!parse_joint_type(attribute(open,"type"),j.type)||j.name.empty())return fail("invalid joint name/type",error);j.parent_link=attribute(first_tag(b,"parent"),"link");j.child_link=attribute(first_tag(b,"child"),"link");if(j.parent_link.empty()||j.child_link.empty())return fail("joint parent/child is missing",error);if(!parse_origin(b,j.origin,error)||!parse_axis(b,j.axis,j.type,error)||!parse_limit(b,j.limits,j.type,error))return false;c.joints.push_back(j);}
    std::vector<std::string> children;for(const auto& j:c.joints){if(!contains(c.links,j.parent_link)||!contains(c.links,j.child_link))return fail("joint references an unknown link",error);if(contains(children,j.child_link))return fail("link has multiple parent joints",error);children.push_back(j.child_link);}
    std::vector<std::string> roots;for(const auto& l:c.links)if(!contains(children,l))roots.push_back(l);if(roots.size()!=1U)return fail("URDF is not a single-root serial tree",error);c.root_link=roots.front();
    std::vector<RobotJoint> ordered;ordered.reserve(c.joints.size());std::string current=c.root_link;while(ordered.size()<c.joints.size()){auto it=std::find_if(c.joints.begin(),c.joints.end(),[&](const RobotJoint& j){return j.parent_link==current;});if(it==c.joints.end())return fail("URDF is not a single serial chain",error);ordered.push_back(*it);current=it->child_link;}c.joints=std::move(ordered);c.tip_link=current;if(!c.valid(error))return false;out=std::move(c);return true;
  }
 private:
  static bool fail(const char* m,std::string* e){if(e)*e=m;return false;} static bool contains(const std::vector<std::string>& v,const std::string& x){return std::find(v.begin(),v.end(),x)!=v.end();}
  static std::string first_tag(const std::string& s,const std::string& n){const std::regex p("<"+n+R"(\b[^>]*>)");std::smatch m;return std::regex_search(s,m,p)?m.str(0):std::string{};}
  static std::vector<std::string> all_tags(const std::string& s,const std::string& n){std::vector<std::string> r;const std::regex p("<"+n+R"(\b[^>]*>)");for(std::sregex_iterator i(s.begin(),s.end(),p),e;i!=e;++i)r.push_back(i->str());return r;}
  static std::vector<std::string> all_elements(const std::string& s,const std::string& n){std::vector<std::string> r;const std::regex p("<"+n+R"(\b[^>]*>[\s\S]*?</)"+n+">)",std::regex::ECMAScript|std::regex::icase);for(std::sregex_iterator i(s.begin(),s.end(),p),e;i!=e;++i)r.push_back(i->str());return r;}
  static std::string attribute(const std::string& tag,const std::string& n){const std::regex p(n+R"(\s*=\s*[\"]([^\"]*)[\"])");std::smatch m;return std::regex_search(tag,m,p)?m[1].str():std::string{};}
  static bool vec3(const std::string& tag,const char* n,double v[3]){std::istringstream ss(attribute(tag,n));return static_cast<bool>(ss>>v[0]>>v[1]>>v[2])&&std::isfinite(v[0])&&std::isfinite(v[1])&&std::isfinite(v[2]);}
  static bool parse_origin(const std::string& b,geometry::Pose3D& p,std::string* e){const std::string t=first_tag(b,"origin");if(t.empty())return true;double x[3]{},r[3]{};if(!attribute(t,"xyz").empty()&&!vec3(t,"xyz",x))return fail("invalid origin xyz",e);if(!attribute(t,"rpy").empty()&&!vec3(t,"rpy",r))return fail("invalid origin rpy",e);const double cr=std::cos(r[0]/2),sr=std::sin(r[0]/2),cp=std::cos(r[1]/2),sp=std::sin(r[1]/2),cy=std::cos(r[2]/2),sy=std::sin(r[2]/2);p.position={x[0],x[1],x[2]};p.rotation=geometry::normalized({cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy});return true;}
  static bool parse_axis(const std::string& b,geometry::Vec3& a,JointType t,std::string* e){const std::string tag=first_tag(b,"axis");if(tag.empty()){a={0,0,1};return true;}double v[3]{};if(!vec3(tag,"xyz",v))return fail("invalid joint axis",e);a={v[0],v[1],v[2]};const double n=geometry::norm(a);if(t!=JointType::Fixed&&n<=1e-12)return fail("movable joint has zero axis",e);if(n>1e-12)a=(1.0/n)*a;return true;}
  static bool parse_limit(const std::string& b,JointLimits& l,JointType t,std::string* e){const std::string tag=first_tag(b,"limit");if(tag.empty())return true;auto read=[&](const char* n,double& x){const std::string v=attribute(tag,n);if(v.empty())return true;try{x=std::stod(v);}catch(...){return false;}return std::isfinite(x);};if((t==JointType::Revolute||t==JointType::Prismatic)&&(!read("lower",l.lower)||!read("upper",l.upper)))return fail("invalid joint limits",e);if(!read("effort",l.effort)||!read("velocity",l.velocity)||l.lower>l.upper)return fail("invalid joint limits",e);return true;}
  static bool parse_joint_type(const std::string& t,JointType& o){if(t=="fixed")o=JointType::Fixed;else if(t=="revolute")o=JointType::Revolute;else if(t=="continuous")o=JointType::Continuous;else if(t=="prismatic")o=JointType::Prismatic;else return false;return true;}
};
} // namespace mca::kinematics
