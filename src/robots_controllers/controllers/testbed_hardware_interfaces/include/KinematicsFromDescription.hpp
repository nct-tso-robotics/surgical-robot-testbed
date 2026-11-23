#pragma once
#include <rcm_solvers/RobotKinematics.h>
#include <memory>
#include <string>
#include <vector>
namespace testbed_hardware_interfaces {
// URDF parsing belongs at the ROS boundary; the core receives only a KDL chain.
std::unique_ptr<rcm_solvers::RobotKinematics> kinematicsFromDescription(
  const std::string& description, const std::string& base, const std::string& tip,
  const std::vector<std::string>& joint_names);
}
