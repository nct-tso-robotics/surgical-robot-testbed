#pragma once

#include "RobotHardwareInterface.hpp"
#include <vector>
#include <string>
#include <memory>
#include <functional>

namespace testbed_hardware_interfaces
{
    /**
    * @brief Hardware interface for the UR5e robot in ROS2.
    *
    * This class implements the RobotHardwareInterface for the UR5e robot, providing methods to access
    * joint names, state interfaces (position, velocity, effort), command interfaces, and wrench data.
    * It enables controllers to interact with the robot's hardware by retrieving state data and sending commands.
    */
    class Ur5eHardwareInterface : public RobotHardwareInterface
    {
        public:
            Ur5eHardwareInterface();
            ~Ur5eHardwareInterface();

            std::vector<std::string> getJointNames() override;

            std::vector<std::string> getJointsPositionStateInterfacesNames() override;
            std::vector<std::string> getJointsPositionCommandInterfacesNames() override;
            std::vector<std::string> getJointsVelocityStateInterfacesNames() override;
            std::vector<std::string> getJointsVelocityCommandInterfacesNames() override;
            std::vector<std::string> getJointsEffortStateInterfacesNames() override;
            std::vector<std::string> getJointsEffortCommandInterfacesNames() override;
            std::vector<std::string> getWrenchStateInterfacesNames() override;


            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> getStatePositionInterfaces(std::vector<hardware_interface::LoanedStateInterface> & state_interfaces) override;
            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> getStateVelocityInterfaces(std::vector<hardware_interface::LoanedStateInterface> & state_interfaces) override;
            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> getStateEffortInterfaces(std::vector<hardware_interface::LoanedStateInterface> & state_interfaces) override;

            std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> getCommandPositionInterfaces(std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces) override;
            std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> getCommandVelocityInterfaces(std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces) override;
            std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> getCommandEffortInterfaces(std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces) override;

            geometry_msgs::msg::Wrench getWrench(std::vector<hardware_interface::LoanedStateInterface> & state_interfaces) override;

        private:
            std::vector<std::string> getJointsStateInterfacesNames(std::string interface_name);
            std::vector<std::string> getJointsCommandInterfacesNames(std::string interface_name);
            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> getJointsInterfaces(std::vector<hardware_interface::LoanedStateInterface> & state_interfaces, std::vector<std::string> interfaces_names);
            std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> getJointsInterfaces(std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces, std::vector<std::string> interfaces_names);
            std::vector<std::string> joint_names_;
    };
}