#pragma once

#include <memory>
#include <string>
#include <stdexcept>
#include "Ur5eHardwareInterface.hpp"
#include <rcm_solvers/RobotKinematicsFactory.h>

namespace testbed_hardware_interfaces {

class RobotHardwareInterfaceFactory {
public:
    /** @brief Create a RobotHardwareInterface instance for the specified robot type */
    static std::unique_ptr<RobotHardwareInterface> create(rcm_solvers::RobotType robot_type);
};

} // namespace testbed_hardware_interfaces