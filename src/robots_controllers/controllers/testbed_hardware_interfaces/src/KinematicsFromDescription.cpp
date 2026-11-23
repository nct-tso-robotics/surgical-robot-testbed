#include "KinematicsFromDescription.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <stdexcept>
#include <urdf/model.h>
#include <limits>
#include <algorithm>
namespace testbed_hardware_interfaces {
std::unique_ptr<rcm_solvers::RobotKinematics> kinematicsFromDescription(
  const std::string& description, const std::string& base, const std::string& tip,
  const std::vector<std::string>& joint_names) {
  KDL::Tree tree;
  KDL::Chain chain;
  if (description.empty() || !kdl_parser::treeFromString(description, tree) ||
      !tree.getChain(base, tip, chain))
    throw std::invalid_argument("Cannot resolve kinematics chain from controller manager robot_description");
  std::vector<std::string> actual_names;
  for (const auto& segment : chain.segments)
    if (segment.getJoint().getType() != KDL::Joint::None)
      actual_names.push_back(segment.getJoint().getName());
  if (actual_names != joint_names)
    throw std::invalid_argument("Kinematics chain joint order differs from hardware interfaces");
  urdf::Model model;
  if (!model.initString(description)) throw std::invalid_argument("Invalid URDF joint limits");
  std::vector<double> lower, upper, velocity;
  for (const auto& name : joint_names) {
    auto joint=model.getJoint(name);
    if (!joint || !joint->limits) throw std::invalid_argument("Joint velocity limits required in URDF");
    double lo=joint->type==urdf::Joint::CONTINUOUS ? -std::numeric_limits<double>::infinity() : joint->limits->lower;
    double hi=joint->type==urdf::Joint::CONTINUOUS ? std::numeric_limits<double>::infinity() : joint->limits->upper;
    if (joint->safety) {
      lo=std::max(lo,joint->safety->soft_lower_limit);
      hi=std::min(hi,joint->safety->soft_upper_limit);
    }
    lower.push_back(lo); upper.push_back(hi); velocity.push_back(joint->limits->velocity);
  }
  auto result=std::make_unique<rcm_solvers::UR5eKinematics>(chain);
  result->setJointLimits(lower,upper,velocity);
  return result;
}
}
