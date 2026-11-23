#pragma once

#include <memory>
#include <string>
#include <stdexcept>
#include "RobotKinematics.h"

namespace rcm_solvers {

enum class RobotType { UR5E, FRANKA };

/**
 * @brief Convert a string to RobotType enum
 * @param robot_type_str String representation of the robot type (case insensitive)
 * @return RobotType enum value
 * @throw std::runtime_error if the string doesn't match any valid robot type
 */
RobotType string_to_robot_type(const std::string& robot_type_str);

/**
 * @brief Factory that creates the correct RobotKinematics implementation based
 * on the requested RobotType.
 */
class RobotKinematicsFactory {
public:
    /** @brief Create a RobotKinematics instance for the specified robot type */
    static std::unique_ptr<RobotKinematics> create(RobotType robot_type);
};

} // namespace rcm_solvers