#include "RcmBroadcaster.hpp"
#include "KinematicsFromDescription.hpp"
#include <cmath>
#include <stdexcept>

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"
#include <rcm_solvers/RobotKinematicsFactory.h>

/**
 * @class RcmBroadcaster
 * @brief A broadcaster for robot state information, including joint states, end-effector pose, and tip point data.
 *
 * This class is responsible for reading robot state from hardware interfaces and publishing it in a standardized message format.
 * It handles initialization, configuration, activation, deactivation, and periodic updates to broadcast the robot's state.
 */
namespace testbed_broadcasters
{

/**
 * @brief Default constructor for RcmBroadcaster.
 */
RcmBroadcaster::RcmBroadcaster() {}

/**
 * @brief Reads parameters from the parameter server and initializes class attributes.
 *
 * This method retrieves parameters such as robot type, end-effector frame tip point, and RCM point.
 * It also creates the robot hardware interface and kinematics based on the robot type.
 */
void RcmBroadcaster::readParamsAndInitializeAttributes() {
  this->params_ = this->params_listener_->get_params();
  rcm_solvers::RobotType robot_type = rcm_solvers::string_to_robot_type(this->params_.robot_type);
  this->robot_hardware_interface_ = testbed_hardware_interfaces::RobotHardwareInterfaceFactory::create(robot_type);
  if (params_.ee_frame_tip_point.size()!=3 || params_.ee_frame_rcm_point.size()!=3)
    throw std::invalid_argument("Tip/pivot must have three coordinates");
  std::vector<double> ee_frame_tip_point = this->params_.ee_frame_tip_point;
  this->ee_frame_tip_point = Eigen::Vector3d(ee_frame_tip_point[0], ee_frame_tip_point[1], ee_frame_tip_point[2]);
  std::vector<double> ee_frame_rcm_point = this->params_.ee_frame_rcm_point;
  this->ee_frame_rcm_point = Eigen::Vector3d(ee_frame_rcm_point[0], ee_frame_rcm_point[1], ee_frame_rcm_point[2]);
  if (!this->ee_frame_tip_point.allFinite() || !this->ee_frame_rcm_point.allFinite() ||
      (this->ee_frame_tip_point-this->ee_frame_rcm_point).norm()<1e-6)
    throw std::invalid_argument("Tip/pivot must be finite and distinct");
}

/**
 * @brief Initializes the broadcaster by setting up the parameter listener.
 *
 * @return CallbackReturn::SUCCESS if initialization is successful, CallbackReturn::ERROR otherwise.
 */
controller_interface::CallbackReturn RcmBroadcaster::on_init() {
  try {
    // Create the parameter listener
    this->params_listener_ = std::make_shared<rcm_broadcaster::ParamListener>(get_node());
    this->readParamsAndInitializeAttributes();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize parameters: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Configures the broadcaster by setting up the publisher for robot state messages.
 *
 * @param previous_state The previous lifecycle state.
 * @return CallbackReturn::SUCCESS if configuration is successful.
 */
controller_interface::CallbackReturn RcmBroadcaster::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  try {
    initialized = false;
    base_frame_id_ = params_.frame_prefix + params_.base_frame;
    robot_kinematics = testbed_hardware_interfaces::kinematicsFromDescription(
      get_robot_description(), params_.base_frame, params_.tip_frame, robot_hardware_interface_->getJointNames());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  this->robot_state_publisher_ = get_node()->create_publisher<testbed_msgs::msg::RobotState>(
      params_.robot_state_topic, rclcpp::QoS(10).reliable());

  this->realtime_robot_state_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<testbed_msgs::msg::RobotState>>(
      this->robot_state_publisher_);

  auto & robot_state_msg = realtime_robot_state_publisher_->msg_;
  robot_state_msg.joints_state.name.reserve(this->robot_kinematics->getNumJoints());
  robot_state_msg.joints_state.position.reserve(this->robot_kinematics->getNumJoints());
  robot_state_msg.joints_state.velocity.reserve(this->robot_kinematics->getNumJoints());
  robot_state_msg.joints_state.effort.reserve(this->robot_kinematics->getNumJoints());

  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Returns the command interface configuration.
 *
 * Since this broadcaster does not handle commands, it returns a configuration with type NONE.
 *
 * @return InterfaceConfiguration with type NONE.
 */
controller_interface::InterfaceConfiguration RcmBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE
  };
}

/**
 * @brief Returns the state interface configuration.
 *
 * This method collects all state interface names from the robot hardware interface,
 * including position, velocity, effort, and wrench interfaces.
 *
 * @return InterfaceConfiguration with individual state interface names.
 */
controller_interface::InterfaceConfiguration RcmBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  auto position_interfaces_names = this->robot_hardware_interface_->getJointsPositionStateInterfacesNames();
  auto velocity_interfaces_names = this->robot_hardware_interface_->getJointsVelocityStateInterfacesNames();
  auto effort_interfaces_names = this->robot_hardware_interface_->getJointsEffortStateInterfacesNames();
  auto wrench_interfaces_names = this->robot_hardware_interface_->getWrenchStateInterfacesNames();

  std::vector<std::string> combined_interfaces_names;
  combined_interfaces_names.insert(combined_interfaces_names.end(), position_interfaces_names.begin(), position_interfaces_names.end());
  combined_interfaces_names.insert(combined_interfaces_names.end(), velocity_interfaces_names.begin(), velocity_interfaces_names.end());
  if (params_.read_effort) combined_interfaces_names.insert(combined_interfaces_names.end(), effort_interfaces_names.begin(), effort_interfaces_names.end());
  if (params_.read_wrench) combined_interfaces_names.insert(combined_interfaces_names.end(), wrench_interfaces_names.begin(), wrench_interfaces_names.end());

  state_interfaces_config.names = combined_interfaces_names;
  return state_interfaces_config;
}

/**
 * @brief Activates the broadcaster by setting up state interfaces.
 *
 * This method clears and repopulates the position, velocity, and effort state interfaces
 * based on the current state interfaces provided by the hardware.
 *
 * @param previous_state The previous lifecycle state.
 * @return CallbackReturn::SUCCESS if activation is successful.
 */
controller_interface::CallbackReturn RcmBroadcaster::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  this->position_state_interfaces_.clear();
  this->velocity_state_interfaces_.clear();
  this->effort_state_interfaces_.clear();
  this->position_state_interfaces_ = this->robot_hardware_interface_->getStatePositionInterfaces(this->state_interfaces_);
  this->velocity_state_interfaces_ = this->robot_hardware_interface_->getStateVelocityInterfaces(this->state_interfaces_);
  this->effort_state_interfaces_ = this->robot_hardware_interface_->getStateEffortInterfaces(this->state_interfaces_);
  const auto count = robot_hardware_interface_->getJointNames().size();
  if (position_state_interfaces_.size()!=count || velocity_state_interfaces_.size()!=count ||
      (params_.read_effort && effort_state_interfaces_.size()!=count))
    return controller_interface::CallbackReturn::ERROR;

  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Deactivates the broadcaster by clearing state interfaces and resetting initialization flag.
 *
 * @param previous_state The previous lifecycle state.
 * @return CallbackReturn::SUCCESS if deactivation is successful.
 */
controller_interface::CallbackReturn RcmBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  this->position_state_interfaces_.clear();
  this->velocity_state_interfaces_.clear();
  this->effort_state_interfaces_.clear();

  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Updates and publishes the robot state.
 *
 * This method reads the current joint states, computes the end-effector pose and velocities,
 * and publishes the robot state message if the state interfaces are available.
 *
 * @param time The current time.
 * @param period The time period since the last update.
 * @return return_type::OK if the update is successful.
 */
controller_interface::return_type RcmBroadcaster::update(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (this->position_state_interfaces_.empty() || this->velocity_state_interfaces_.empty()) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Skipping update: one or more required interfaces are empty (pos=%zu, vel=%zu, eff=%zu)",
      this->position_state_interfaces_.size(),
      this->velocity_state_interfaces_.size(),
      this->effort_state_interfaces_.size());
    return controller_interface::return_type::OK;
  }
  {
    auto joint_names = this->robot_hardware_interface_->getJointNames();
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;
    for (size_t i = 0; i < joint_names.size(); i++) {
      auto position=position_state_interfaces_[i].get().get_optional();
      auto velocity=velocity_state_interfaces_[i].get().get_optional();
      if (!position || !velocity || !std::isfinite(*position) || !std::isfinite(*velocity))
        return controller_interface::return_type::ERROR;
      joint_positions.push_back(*position);
      joint_velocities.push_back(*velocity);
      auto effort=params_.read_effort ? effort_state_interfaces_[i].get().get_optional() : std::optional<double>{};
      joint_efforts.push_back(effort.value_or(std::numeric_limits<double>::quiet_NaN()));
    }

    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = time;
    joint_state_msg.header.frame_id = base_frame_id_;
    joint_state_msg.name.reserve(joint_names.size());
    joint_state_msg.position.reserve(joint_names.size());
    joint_state_msg.velocity.reserve(joint_names.size());
    joint_state_msg.effort.reserve(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); i++) {
      joint_state_msg.name.push_back(joint_names[i]);
      joint_state_msg.position.push_back(joint_positions[i]);
      joint_state_msg.velocity.push_back(joint_velocities[i]);
      joint_state_msg.effort.push_back(joint_efforts[i]);
    }

    auto wrench_msg = this->robot_hardware_interface_->getWrench(this->state_interfaces_);

    Eigen::Matrix4d robot_transform = this->robot_kinematics->calculateFK(joint_positions);

    Eigen::Quaterniond current_ee_orientation(robot_transform.block<3,3>(0,0));
    Eigen::Vector3d current_ee_position(robot_transform.block<3,1>(0,3));
    rcm_solvers::RobotPose current_pose = rcm_solvers::getRobotPose(current_ee_orientation, current_ee_position);
    Eigen::Vector3d robot_space_tip_point = rcm_solvers::pointLocalToGlobal(this->ee_frame_tip_point, current_pose);

    if (!this->initialized) {
      this->robot_space_initial_tip_point = robot_space_tip_point;
      this->robot_space_rcm_point = rcm_solvers::pointLocalToGlobal(this->ee_frame_rcm_point, current_pose);
      this->initial_ee_orientation_ = current_ee_orientation; // Store initial orientation of the end effector for roll computation
      this->initial_shaft_axis_ = (robot_space_tip_point - this->robot_space_rcm_point).normalized(); // Initial shaft axis
      this->previous_roll_ = 0.0;
      this->initialized = true;
    }

    // Calculate Cartesian velocities
    Eigen::MatrixXd jacobian = this->robot_kinematics->getJacobian(joint_positions);
    Eigen::Map<Eigen::VectorXd> joint_velocities_eigen(joint_velocities.data(), joint_velocities.size());
    Eigen::VectorXd cartesian_velocities = jacobian * joint_velocities_eigen;

    // Extract linear and angular velocities
    Eigen::Vector3d ee_linear_velocity = cartesian_velocities.head<3>();
    Eigen::Vector3d ee_angular_velocity = cartesian_velocities.tail<3>();

    geometry_msgs::msg::Pose robot_pose_msg;
    assignVectorToXYZ(robot_pose_msg.position, current_ee_position);

    robot_pose_msg.orientation.x = current_ee_orientation.x();
    robot_pose_msg.orientation.y = current_ee_orientation.y();
    robot_pose_msg.orientation.z = current_ee_orientation.z();
    robot_pose_msg.orientation.w = current_ee_orientation.w();

    geometry_msgs::msg::Twist end_effector_twist_msg;
    assignVectorToXYZ(end_effector_twist_msg.angular, ee_angular_velocity);
    assignVectorToXYZ(end_effector_twist_msg.linear, ee_linear_velocity);

    geometry_msgs::msg::Point relative_tip_position_msg;
    Eigen::Vector3d current_relative_tip = robot_space_tip_point - this->robot_space_initial_tip_point;
    assignVectorToXYZ(relative_tip_position_msg, current_relative_tip);
    geometry_msgs::msg::Point robot_space_initial_tip_position_msg;
    assignVectorToXYZ(robot_space_initial_tip_position_msg, this->robot_space_initial_tip_point);
    geometry_msgs::msg::Point robot_space_tip_position_msg;
    assignVectorToXYZ(robot_space_tip_position_msg, robot_space_tip_point);

    Eigen::Vector3d robot_space_tip_velocity = rcm_solvers::getVelAt(robot_space_tip_point,
                          current_ee_position,
                          ee_linear_velocity,
                          ee_angular_velocity);

    geometry_msgs::msg::Twist robot_space_tip_twist_msg;

    assignVectorToXYZ(robot_space_tip_twist_msg.angular, ee_angular_velocity);
    assignVectorToXYZ(robot_space_tip_twist_msg.linear, robot_space_tip_velocity);

    geometry_msgs::msg::Point robot_space_rcm_position_msg;
    assignVectorToXYZ(robot_space_rcm_position_msg, this->robot_space_rcm_point);

    auto [pitch, yaw, pivot_translation] = rcm_solvers::convertRelativeTipPositionToPYT(
        this->robot_space_rcm_point, this->robot_space_initial_tip_point, current_relative_tip);

    double roll = rcm_solvers::computeRoll(current_ee_orientation,
                          this->initial_ee_orientation_,
                          this->initial_shaft_axis_,
                          robot_space_tip_point,
                          robot_space_rcm_point,
                          this->previous_roll_);
    this->previous_roll_ = roll;

    Eigen::Vector3d pivot_rpy(roll, pitch, yaw);
    geometry_msgs::msg::Point relative_pivot_rpy_msg;
    assignVectorToXYZ(relative_pivot_rpy_msg, pivot_rpy);

    if (this->realtime_robot_state_publisher_ && this->realtime_robot_state_publisher_->trylock()) {
      auto & robot_state_msg = this->realtime_robot_state_publisher_->msg_;

      robot_state_msg.header.stamp = time;
      robot_state_msg.header.frame_id = base_frame_id_;
      robot_state_msg.joints_state = joint_state_msg;
      robot_state_msg.robot_pose = robot_pose_msg;
      robot_state_msg.end_effector_twist = end_effector_twist_msg;
      robot_state_msg.robot_space_tip_position = robot_space_tip_position_msg;
      robot_state_msg.relative_tip_position = relative_tip_position_msg;
      robot_state_msg.robot_space_initial_tip_position = robot_space_initial_tip_position_msg;
      robot_state_msg.robot_space_tip_twist = robot_space_tip_twist_msg;
      robot_state_msg.robot_space_wrench = wrench_msg;
      robot_state_msg.robot_space_rcm_position = robot_space_rcm_position_msg;
      robot_state_msg.relative_pivot_rpy = relative_pivot_rpy_msg;
      robot_state_msg.relative_pivot_translation = pivot_translation;

      this->realtime_robot_state_publisher_->unlockAndPublish();
    }
  }
  return controller_interface::return_type::OK;
}



}  // namespace testbed_broadcasters

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  testbed_broadcasters::RcmBroadcaster, controller_interface::ControllerInterface)
