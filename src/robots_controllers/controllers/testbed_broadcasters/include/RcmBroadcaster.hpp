#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <optional>
#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/version.h>
#include <Eigen/Core>
#include <rcm_solvers/RobotKinematics.h>
#include <testbed_msgs/msg/robot_state.hpp>
#include "testbed_broadcasters/rcm_broadcaster_parameters.hpp"
#include <RobotHardwareInterfaceFactory.hpp>

namespace testbed_broadcasters
{

/**
 * @brief A broadcaster for publishing robot state information for RCM-constrained control.
 */
class RcmBroadcaster : public controller_interface::ControllerInterface
{

public:
  RcmBroadcaster();
  ~RcmBroadcaster() = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;


private:

  void readParamsAndInitializeAttributes();

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> position_state_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> velocity_state_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> effort_state_interfaces_;

  std::shared_ptr<testbed_hardware_interfaces::RobotHardwareInterface> robot_hardware_interface_;
  Eigen::Vector3d ee_frame_tip_point; // Tip of the instrument in end-effector frame
  Eigen::Vector3d robot_space_initial_tip_point; // Tip of the instrument in robot frame
  Eigen::Vector3d ee_frame_rcm_point; // RCM point in end-effector frame
  Eigen::Vector3d robot_space_rcm_point; // RCM point in robot frame
  Eigen::Quaterniond initial_ee_orientation_; // Initial end-effector orientation
  Eigen::Vector3d initial_shaft_axis_;        // Initial shaft axis
  double previous_roll_{0.0};  // Last computed roll angle, used for unwrapping continuity.
  std::unique_ptr<rcm_solvers::RobotKinematics> robot_kinematics;
  bool initialized = false;

  // publishers
  std::shared_ptr<rclcpp::Publisher<testbed_msgs::msg::RobotState>> robot_state_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<testbed_msgs::msg::RobotState>>
    realtime_robot_state_publisher_;

  rcm_broadcaster::Params params_;
  std::string base_frame_id_;
  std::shared_ptr<rcm_broadcaster::ParamListener> params_listener_;
};


/**
 * @brief Assigns the components of an Eigen::Vector3d to the x, y, z fields of a ROS message field.
 *
 * This template function simplifies transferring data from an Eigen::Vector3d to a ROS message field
 * that has x, y, z components, such as geometry_msgs::Point or geometry_msgs::Vector3.
 *
 * @tparam T The type of the ROS message field, which must have x, y, z members.
 * @param field The ROS message field to be set.
 * @param vector The Eigen::Vector3d containing the values to assign.
 *
 * @note This function assumes that the field has public x, y, z members and that they are of a type compatible with double.
 */
template <typename T>
void assignVectorToXYZ(T& field, const Eigen::Vector3d& vector) {
    field.x = vector[0];
    field.y = vector[1];
    field.z = vector[2];
}

}  // namespace testbed_broadcasters
