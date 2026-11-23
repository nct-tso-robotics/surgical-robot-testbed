#include "RobotHardwareInterfaceFactory.hpp"
#include <algorithm>

namespace testbed_hardware_interfaces {

/**
* @brief Creates a robot hardware interface based on the specified robot type.
*
* This method returns a unique pointer to a RobotHardwareInterface instance
* corresponding to the given robot type. Currently, only UR5E is supported.
*
* @param robot_type The type of robot for which to create the hardware interface.
* @return A unique pointer to the created RobotHardwareInterface.
* @throws std::runtime_error if the robot type is not supported.
*/
std::unique_ptr<RobotHardwareInterface> RobotHardwareInterfaceFactory::create(rcm_solvers::RobotType robot_type) {

    std::unique_ptr<RobotHardwareInterface> robot_hardware_interface;
    if (robot_type == rcm_solvers::RobotType::UR5E) {
        robot_hardware_interface = std::make_unique<testbed_hardware_interfaces::Ur5eHardwareInterface>();
    }
    else {
        throw std::runtime_error("Invalid robot type. Supported types are: UR5E");
    }

    return robot_hardware_interface;
}

} // namespace testbed_hardware_interfaces