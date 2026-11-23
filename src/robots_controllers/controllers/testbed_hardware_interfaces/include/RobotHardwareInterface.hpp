#pragma once

#include <vector>
#include <string>
#include <functional>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench.hpp>

namespace testbed_hardware_interfaces
{
    /**
    * @brief Abstract interface for robot hardware in ROS2.
    *
    * This class provides methods to access joint states (position, velocity, effort), send joint commands,
    * and retrieve wrench data, enabling controller-hardware interaction.
    */
    class RobotHardwareInterface
    {
        public:
            virtual ~RobotHardwareInterface() = default;

            virtual std::vector<std::string> getJointNames() = 0;

            virtual std::vector<std::string> getJointsPositionStateInterfacesNames() = 0;
            virtual std::vector<std::string> getJointsPositionCommandInterfacesNames() = 0;
            virtual std::vector<std::string> getJointsVelocityStateInterfacesNames() = 0;
            virtual std::vector<std::string> getJointsVelocityCommandInterfacesNames() = 0;
            virtual std::vector<std::string> getJointsEffortStateInterfacesNames() = 0;
            virtual std::vector<std::string> getJointsEffortCommandInterfacesNames() = 0;
            virtual std::vector<std::string> getWrenchStateInterfacesNames() = 0;

            virtual std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> getStatePositionInterfaces(std::vector<hardware_interface::LoanedStateInterface> & state_interfaces) = 0;
            virtual std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> getStateVelocityInterfaces(std::vector<hardware_interface::LoanedStateInterface> & state_interfaces) = 0;
            virtual std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> getStateEffortInterfaces(std::vector<hardware_interface::LoanedStateInterface> & state_interfaces) = 0;

            virtual std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> getCommandPositionInterfaces(std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces) = 0;
            virtual std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> getCommandVelocityInterfaces(std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces) = 0;
            virtual std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> getCommandEffortInterfaces(std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces) = 0;

            virtual geometry_msgs::msg::Wrench getWrench(std::vector<hardware_interface::LoanedStateInterface> & state_interfaces) = 0;
    };
}