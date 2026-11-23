#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <rcm_solvers/JointSolver.h>
#include <testbed_msgs/msg/target_tip_pose.hpp>
#include <RobotHardwareInterfaceFactory.hpp>
#include "testbed_joint_rcm_controller/joint_rcm_controller_parameters.hpp"
namespace testbed_joint_rcm_controller {
class JointRcmController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
private:
  struct Command {
    Eigen::Vector3d delta;
    double roll, duration;
    std::int64_t received;
    std::uint64_t generation;
  };
  struct Source { std::uint64_t sequence{0}; std::int64_t stamp{0}; };
  static std::int64_t steadyNow();
  void receive(const testbed_msgs::msg::TargetTipPose::SharedPtr& msg);
  void zeroCommands();
  bool initialized_{false};
  std::atomic<bool> active_{false};
  std::int64_t activated_at_{0};
  std::int64_t last_received_{0};
  std::uint64_t generation_{0}, processed_{0};
  std::unordered_map<std::string, Source> sources_; // callback-thread only, bounded
  realtime_tools::RealtimeBuffer<std::shared_ptr<const Command>> command_buffer_;
  rclcpp::Subscription<testbed_msgs::msg::TargetTipPose>::SharedPtr subscription_;
  std::unique_ptr<rcm_solvers::JointSolver> solver_;
  std::unique_ptr<testbed_hardware_interfaces::RobotHardwareInterface> hardware_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> commands_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> positions_, velocities_;
  std::vector<double> q_, dq_;
  joint_rcm_controller::Params params_;
  std::string base_frame_id_;
  std::shared_ptr<joint_rcm_controller::ParamListener> listener_;
};
}
