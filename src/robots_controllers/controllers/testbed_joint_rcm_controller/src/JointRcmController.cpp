#include "JointRcmController.hpp"
#include "KinematicsFromDescription.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace testbed_joint_rcm_controller {
using CallbackReturn = controller_interface::CallbackReturn;
std::int64_t JointRcmController::steadyNow() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
CallbackReturn JointRcmController::on_init() {
  try {
    listener_ = std::make_shared<joint_rcm_controller::ParamListener>(get_node());
    params_ = listener_->get_params();
    hardware_ = testbed_hardware_interfaces::RobotHardwareInterfaceFactory::create(
      rcm_solvers::string_to_robot_type(params_.robot_type));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}
controller_interface::InterfaceConfiguration JointRcmController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          hardware_->getJointsVelocityCommandInterfacesNames()};
}
controller_interface::InterfaceConfiguration JointRcmController::state_interface_configuration() const {
  auto names = hardware_->getJointsPositionStateInterfacesNames();
  auto velocities = hardware_->getJointsVelocityStateInterfacesNames();
  names.insert(names.end(), velocities.begin(), velocities.end());
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}
CallbackReturn JointRcmController::on_configure(const rclcpp_lifecycle::State&) {
  try {
    params_ = listener_->get_params();
    initialized_ = false;
    base_frame_id_ = params_.frame_prefix + params_.base_frame;
    if (params_.ee_frame_tip_point.size()!=3 || params_.ee_frame_rcm_point.size()!=3)
      throw std::invalid_argument("Tip and pivot must have three coordinates");
    for (double value : {params_.max_command_age, params_.command_timeout, params_.max_delta_position,
                        params_.max_delta_roll, params_.max_movement_time, params_.joint_velocity_limit})
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Command limits must be positive");
    solver_ = std::make_unique<rcm_solvers::JointSolver>(
      testbed_hardware_interfaces::kinematicsFromDescription(get_robot_description(),
        params_.base_frame, params_.tip_frame, hardware_->getJointNames()));
    solver_->setLocalTipPoint(Eigen::Map<const Eigen::Vector3d>(params_.ee_frame_tip_point.data()));
    solver_->setLocalPivotPoint(Eigen::Map<const Eigen::Vector3d>(params_.ee_frame_rcm_point.data()));
    solver_->setCartesianVelocityLimit(params_.cartesian_velocity_limit);
    solver_->setCartesianAccelerationLimit(params_.cartesian_acceleration_limit);
    solver_->setCartesianJerkLimit(params_.cartesian_jerk_limit);
    solver_->setAngularLimits(params_.angular_velocity_limit, params_.angular_acceleration_limit, params_.angular_jerk_limit);
    q_.resize(hardware_->getJointNames().size()); dq_.resize(q_.size());
    subscription_ = get_node()->create_subscription<testbed_msgs::msg::TargetTipPose>(
      params_.target_tip_pose_topic, rclcpp::QoS(1).reliable(),
      [this](testbed_msgs::msg::TargetTipPose::SharedPtr msg) { receive(msg); });
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}
void JointRcmController::receive(const testbed_msgs::msg::TargetTipPose::SharedPtr& msg) {
  const auto received = steadyNow();
  if (!active_.load()) return;
  const Eigen::Vector3d delta(msg->target_delta_position.x, msg->target_delta_position.y, msg->target_delta_position.z);
  if (!delta.allFinite() || !std::isfinite(msg->target_delta_roll) || !std::isfinite(msg->movement_time) ||
      delta.norm()>params_.max_delta_position || std::abs(msg->target_delta_roll)>params_.max_delta_roll ||
      msg->movement_time<=0 || msg->movement_time>params_.max_movement_time ||
      (!msg->header.frame_id.empty() && msg->header.frame_id!=base_frame_id_) ||
      msg->header.stamp.sec<0 || msg->header.stamp.nanosec>=1000000000u ||
      msg->source_id.size()>128 || (msg->sequence>0 && msg->source_id.empty())) return;
  const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
  if (stamp!=0) {
    const double age=(get_node()->now().nanoseconds()-stamp)*1e-9;
    if (age>params_.max_command_age || age < -0.1) return;
  }
  auto existing=sources_.find(msg->source_id);
  if (existing==sources_.end() && sources_.size()>=64) return;
  auto& source=sources_[msg->source_id];
  if (msg->sequence>0) {
    if (msg->sequence<=source.sequence) return;
    source.sequence=msg->sequence;
  } else if (stamp!=0) {
    if (stamp<=source.stamp) return;
  }
  source.stamp=stamp;
  command_buffer_.writeFromNonRT(std::make_shared<const Command>(Command{
    delta, msg->target_delta_roll, msg->movement_time, received, ++generation_}));
}
CallbackReturn JointRcmController::on_activate(const rclcpp_lifecycle::State&) {
  positions_=hardware_->getStatePositionInterfaces(state_interfaces_);
  velocities_=hardware_->getStateVelocityInterfaces(state_interfaces_);
  commands_=hardware_->getCommandVelocityInterfaces(command_interfaces_);
  if (positions_.size()!=q_.size() || velocities_.size()!=q_.size() || commands_.size()!=q_.size())
    return CallbackReturn::ERROR;
  command_buffer_.reset(); processed_=0; last_received_=0;
  solver_->stop(); zeroCommands();
  activated_at_=steadyNow(); active_.store(true);
  return CallbackReturn::SUCCESS;
}
void JointRcmController::zeroCommands() {
  for (auto& command : commands_) (void)command.get().set_value(0.0);
}
CallbackReturn JointRcmController::on_deactivate(const rclcpp_lifecycle::State&) {
  active_.store(false); zeroCommands(); solver_->stop();
  positions_.clear(); velocities_.clear(); commands_.clear();
  return CallbackReturn::SUCCESS;
}
controller_interface::return_type JointRcmController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  auto stop = [this]() { zeroCommands(); solver_->stop(); };
  if (!active_.load()) { zeroCommands(); return controller_interface::return_type::OK; }
  try {
    if (period.seconds()<=0 || period.seconds()>0.1) { stop(); return controller_interface::return_type::OK; }
    for (size_t i=0;i<q_.size();++i) {
      auto q=positions_[i].get().get_optional(), dq=velocities_[i].get().get_optional();
      if (!q || !dq || !std::isfinite(*q) || !std::isfinite(*dq)) {
        stop(); return controller_interface::return_type::ERROR;
      }
      q_[i]=*q; dq_[i]=*dq;
    }
    const auto transform=solver_->getRobotKinematics().calculateFK(q_);
    auto pose=rcm_solvers::getRobotPose(Eigen::Quaterniond(transform.block<3,3>(0,0)),
                                     Eigen::Vector3d(transform.block<3,1>(0,3)));
    if (!initialized_) { solver_->starting(pose); initialized_=true; }
    const auto now=steadyNow();
    auto latest=command_buffer_.readFromRT();
    if (latest && *latest && (*latest)->generation!=processed_) {
      const auto& command=**latest; processed_=command.generation;
      if (command.received>=activated_at_ && (now-command.received)*1e-9<=params_.max_command_age) {
        solver_->updateTargetPose(command.delta,command.roll,command.duration);
        last_received_=command.received;
      }
    }
    if (!last_received_ || (now-last_received_)*1e-9>params_.command_timeout) {
      stop(); return controller_interface::return_type::OK;
    }
    auto output=solver_->update(pose,q_,dq_,time.seconds(),period.seconds());
    if (output.size()!=static_cast<Eigen::Index>(commands_.size()) || !output.allFinite()) {
      stop(); return controller_interface::return_type::ERROR;
    }
    const auto velocity_limits=solver_->getRobotKinematics().getJointsVelocityLimits();
    const auto position_limits=solver_->getRobotKinematics().getJointsPositionLimits();
    double scale=1.0;
    for (size_t i=0;i<commands_.size();++i) {
      const double limit=std::min(params_.joint_velocity_limit,velocity_limits[i]);
      if (std::abs(output[i])>limit) scale=std::min(scale,limit/std::abs(output[i]));
    }
    output*=scale;
    for (size_t i=0;i<commands_.size();++i) {
      const double next=q_[i]+output[i]*period.seconds();
      if ((next<position_limits.first[i] && output[i]<0) || (next>position_limits.second[i] && output[i]>0)) {
        stop(); return controller_interface::return_type::ERROR;
      }
    }
    for (size_t i=0;i<commands_.size();++i)
      if (!commands_[i].get().set_value(output[i])) { stop(); return controller_interface::return_type::ERROR; }
  } catch (const std::exception& e) {
    stop(); RCLCPP_ERROR(get_node()->get_logger(), "Control update failed: %s", e.what());
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(testbed_joint_rcm_controller::JointRcmController, controller_interface::ControllerInterface)
