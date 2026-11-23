#include "Ur5eHardwareInterface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <iostream>
#include <limits>

namespace testbed_hardware_interfaces
{
    /**
     * @brief Hardware interface for the UR5e robot.
     *
     * This class provides methods to access the joint names, state interfaces,
     * command interfaces, and wrench data of the UR5e robot.
     */

    Ur5eHardwareInterface::Ur5eHardwareInterface()
    {
        this->joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    }

    Ur5eHardwareInterface::~Ur5eHardwareInterface() {}

    /**
    * @brief Gets the names of the robot's joints.
    *
    * @return A vector of strings containing the joint names.
    */
    std::vector<std::string> Ur5eHardwareInterface::getJointNames()
    {
        return this->joint_names_;
    }

    /**
    * @brief Gets the names of the state interfaces for the joints.
    *
    * @param interface_name The type of interface (e.g., "position", "velocity", "effort").
    * @return A vector of strings with the full interface names (e.g., "shoulder_pan_joint/position").
    */
    std::vector<std::string> Ur5eHardwareInterface::getJointsStateInterfacesNames(std::string interface_name)
    {
        std::vector<std::string> interfaces_names;
        for (const auto & joint_name : this->joint_names_) {
            interfaces_names.push_back(joint_name + "/" + interface_name);
        }
        return interfaces_names;
    }

    /**
    * @brief Gets the names of the command interfaces for the joints.
    *
    * @param interface_name The type of interface (e.g., "position", "velocity", "effort").
    * @return A vector of strings with the full interface names (e.g., "shoulder_pan_joint/velocity").
    */
    std::vector<std::string> Ur5eHardwareInterface::getJointsCommandInterfacesNames(std::string interface_name)
    {
        return this->getJointsStateInterfacesNames(interface_name);
    }

    /**
    * @brief Gets the names of the position state interfaces for the joints.
    *
    * @return A vector of strings with the position state interface names.
    */
    std::vector<std::string> Ur5eHardwareInterface::getJointsPositionStateInterfacesNames()
    {
        return this->getJointsStateInterfacesNames(hardware_interface::HW_IF_POSITION);
    }

    /**
    * @brief Gets the names of the position command interfaces for the joints.
    *
    * @return A vector of strings with the position command interface names.
    */
    std::vector<std::string> Ur5eHardwareInterface::getJointsPositionCommandInterfacesNames()
    {
        return this->getJointsCommandInterfacesNames(hardware_interface::HW_IF_POSITION);
    }


    /**
    * @brief Gets the names of the velocity state interfaces for the joints.
    *
    * @return A vector of strings with the velocity state interface names.
    */
    std::vector<std::string> Ur5eHardwareInterface::getJointsVelocityStateInterfacesNames()
    {
        return this->getJointsStateInterfacesNames(hardware_interface::HW_IF_VELOCITY);
    }

    /**
    * @brief Gets the names of the velocity command interfaces for the joints.
    *
    * @return A vector of strings with the velocity command interface names.
    */
    std::vector<std::string> Ur5eHardwareInterface::getJointsVelocityCommandInterfacesNames()
    {
        return this->getJointsCommandInterfacesNames(hardware_interface::HW_IF_VELOCITY);
    }

    /**
    * @brief Gets the names of the effort state interfaces for the joints.
    *
    * @return A vector of strings with the effort state interface names.
    */
    std::vector<std::string> Ur5eHardwareInterface::getJointsEffortStateInterfacesNames()
    {
        return this->getJointsStateInterfacesNames(hardware_interface::HW_IF_EFFORT);
    }

    /**
    * @brief Gets the names of the effort command interfaces for the joints.
    *
    * @return A vector of strings with the effort command interface names.
    */
    std::vector<std::string> Ur5eHardwareInterface::getJointsEffortCommandInterfacesNames()
    {
        return this->getJointsCommandInterfacesNames(hardware_interface::HW_IF_EFFORT);
    }

    /**
    * @brief Gets the names of the wrench state interfaces.
    *
    * @return A vector of strings with the wrench interface names (force and torque components).
    */
    std::vector<std::string> Ur5eHardwareInterface::getWrenchStateInterfacesNames()
    {
        return {
            "tcp_fts_sensor/force.x",
            "tcp_fts_sensor/force.y",
            "tcp_fts_sensor/force.z",
            "tcp_fts_sensor/torque.x",
            "tcp_fts_sensor/torque.y",
            "tcp_fts_sensor/torque.z"
        };
    }

    /**
    * @brief Retrieves references to the specified joint state interfaces.
    *
    * @param state_interfaces A vector of all available state interfaces.
    * @param interfaces_names A vector of names of the interfaces to retrieve.
    * @return A vector of references to the matching LoanedStateInterface objects. If an interface is not found, it is omitted.
    */
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> Ur5eHardwareInterface::getJointsInterfaces(std::vector<hardware_interface::LoanedStateInterface> & state_interfaces, std::vector<std::string> interfaces_names)
    {
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> interfaces;
        for (const auto & interface_name : interfaces_names) {
            for (auto & state_interface : state_interfaces) {
                if (state_interface.get_name() == interface_name) {
                    interfaces.push_back(std::ref(state_interface));
                    break;
                }
            }
        }
        return interfaces;
    }

    /**
    * @brief Retrieves references to the specified joint command interfaces.
    *
    * @param command_interfaces A vector of all available command interfaces.
    * @param interfaces_names A vector of names of the interfaces to retrieve.
    * @return A vector of references to the matching LoanedCommandInterface objects. If an interface is not found, it is omitted.
    */
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> Ur5eHardwareInterface::getJointsInterfaces(std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces, std::vector<std::string> interfaces_names)
    {
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> interfaces;
        for (const auto & interface_name : interfaces_names) {
            for (auto & command_interface : command_interfaces) {
                if (command_interface.get_name() == interface_name) {
                    interfaces.push_back(std::ref(command_interface));
                    break;
                }
            }
        }
        return interfaces;
    }

    /**
    * @brief Gets references to the position state interfaces.
    *
    * @param state_interfaces A vector of all available state interfaces.
    * @return A vector of references to the position state interfaces.
    */
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> Ur5eHardwareInterface::getStatePositionInterfaces(std::vector<hardware_interface::LoanedStateInterface> & state_interfaces)
    {
        return this->getJointsInterfaces(state_interfaces, this->getJointsPositionStateInterfacesNames());
    }

    /**
    * @brief Gets references to the velocity state interfaces.
    *
    * @param state_interfaces A vector of all available state interfaces.
    * @return A vector of references to the velocity state interfaces.
    */
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> Ur5eHardwareInterface::getStateVelocityInterfaces(std::vector<hardware_interface::LoanedStateInterface> & state_interfaces)
    {
        return this->getJointsInterfaces(state_interfaces, this->getJointsVelocityStateInterfacesNames());
    }

    /**
    * @brief Gets references to the effort state interfaces.
    *
    * @param state_interfaces A vector of all available state interfaces.
    * @return A vector of references to the effort state interfaces.
    */
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> Ur5eHardwareInterface::getStateEffortInterfaces(std::vector<hardware_interface::LoanedStateInterface> & state_interfaces)
    {
        return this->getJointsInterfaces(state_interfaces, this->getJointsEffortStateInterfacesNames());
    }

    /**
    * @brief Gets references to the position command interfaces.
    *
    * @param command_interfaces A vector of all available command interfaces.
    * @return A vector of references to the position command interfaces.
    */
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> Ur5eHardwareInterface::getCommandPositionInterfaces(std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces)
    {
        return this->getJointsInterfaces(command_interfaces, this->getJointsPositionCommandInterfacesNames());
    }

    /**
    * @brief Gets references to the velocity command interfaces.
    *
    * @param command_interfaces A vector of all available command interfaces.
    * @return A vector of references to the velocity command interfaces.
    */
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> Ur5eHardwareInterface::getCommandVelocityInterfaces(std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces)
    {
        return this->getJointsInterfaces(command_interfaces, this->getJointsVelocityCommandInterfacesNames());
    }

    /**
    * @brief Gets references to the effort command interfaces.
    *
    * @param command_interfaces A vector of all available command interfaces.
    * @return A vector of references to the effort command interfaces.
    */
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> Ur5eHardwareInterface::getCommandEffortInterfaces(std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces)
    {
        return this->getJointsInterfaces(command_interfaces, this->getJointsEffortCommandInterfacesNames());
    }

    /**
    * @brief Retrieves the current wrench (force and torque) data.
    *
    * This method collects the wrench data from the state interfaces and returns it as a geometry_msgs::msg::Wrench message.
    * It assumes that all six wrench interfaces (force.x, force.y, force.z, torque.x, torque.y, torque.z) are present
    * in the state_interfaces in the order specified by getWrenchStateInterfacesNames(). If any interface is missing,
    * the wrench vector may have fewer elements, potentially leading to an out-of-range access when assigning values.
    *
    * @param state_interfaces A vector of all available state interfaces.
    * @return A Wrench message containing the force and torque values.
    */
    geometry_msgs::msg::Wrench Ur5eHardwareInterface::getWrench(std::vector<hardware_interface::LoanedStateInterface> & state_interfaces)
    {
        const auto names = getWrenchStateInterfacesNames();
        // Missing wrench sensors are explicitly unavailable, never fabricated zeros.
        std::vector<double> wrench(6, std::numeric_limits<double>::quiet_NaN());
        for (size_t i = 0; i < names.size(); ++i)
            for (const auto& iface : state_interfaces)
                if (iface.get_name() == names[i]) {
                    if (auto value = iface.get_optional()) wrench[i] = *value;
                    break;
                }

        geometry_msgs::msg::Wrench wrench_msg;
        wrench_msg.force.x = wrench[0];
        wrench_msg.force.y = wrench[1];
        wrench_msg.force.z = wrench[2];
        wrench_msg.torque.x = wrench[3];
        wrench_msg.torque.y = wrench[4];
        wrench_msg.torque.z = wrench[5];
        return wrench_msg;
    }
}
