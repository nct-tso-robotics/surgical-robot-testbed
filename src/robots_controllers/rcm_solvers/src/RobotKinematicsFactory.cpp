#include "RobotKinematicsFactory.h"
#include <algorithm>

namespace rcm_solvers {

RobotType string_to_robot_type(const std::string& robot_type_str) {
  std::string lower_str = robot_type_str;
  std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);

  if (lower_str == "ur5e") {
    return RobotType::UR5E;
  } else if (lower_str == "franka") {
    return RobotType::FRANKA;
  }
  throw std::runtime_error("Invalid robot type: " + robot_type_str + ". Supported types are: ur5e, franka");
}

/**
 * @brief Create a concrete robot kinematics instance for the specified robot type
 *
 * Factory method that instantiates the appropriate robot kinematics implementation based on
 * the requested robot type. Currently supports UR5E and FRANKA robot types.
 *
 * @param robot_type Enum specifying which robot kinematics to create
 * @return std::unique_ptr<RobotKinematics> Smart pointer to the newly created robot kinematics
 */
std::unique_ptr<RobotKinematics> RobotKinematicsFactory::create(rcm_solvers::RobotType robot_type) {

    if (robot_type == rcm_solvers::RobotType::UR5E) {
        return std::make_unique<rcm_solvers::UR5eKinematics>();
    } else if (robot_type == rcm_solvers::RobotType::FRANKA) {
#if TESTBED_WITH_FRANKA
        return std::make_unique<rcm_solvers::FrankaPandaKinematics>();
#else
        throw std::runtime_error("Franka core support disabled; build with TESTBED_WITH_FRANKA=ON");
#endif
    }
    throw std::runtime_error("Unknown robot type");
}

} // namespace rcm_solvers