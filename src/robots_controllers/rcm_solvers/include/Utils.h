#pragma once

#include <array>
#include <string>
#include <utility>   // for std::pair
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>

namespace rcm_solvers {

/**
 * @brief Compact representation of a 6-DoF end-effector pose.
 *
 * The pose is stored as a quaternion (orientation) and a position vector
 * expressed in the base / world frame:
 *   • orientation – unit quaternion \f$q = (w,x,y,z)\f$
 *   • position    – 3-vector \f$p = (x,y,z)\f$
 *
 * The convention throughout the library is **world-frame position** and
 * **right-handed / Z-up** orientation (same as ROS REP-103).  All helpers in
 * Utils.h expect / return data using that convention.
 */
struct RobotPose {
    Eigen::Quaterniond orientation;
    Eigen::Vector3d position;
};

/**
 * @brief Convert a 4×4 column-major transform into a RobotPose.
 *
 * @param robot_transform Homogeneous transform in row-major order as used by
 *        many robot drivers – 16 consecutive doubles (R00 … R23).
 * @return RobotPose Resulting pose containing position and orientation
 */
RobotPose getRobotPose(const std::array<double,16>& robot_transform);

/**
 * @brief Create a RobotPose from explicit quaternion + position.
 *
 * @param orientation Quaternion representing the orientation
 * @param position Vector representing the position
 * @return RobotPose Combined pose
 */
RobotPose getRobotPose(const Eigen::Quaterniond& orientation,
                       const Eigen::Vector3d& position);

/**
 * @brief Transform a point expressed in the **end-effector local frame** to
 *        the **world frame**.
 *
 * @param local_point Point expressed in the local frame
 * @param robot_pose Robot pose in the global frame
 * @return Eigen::Vector3d Transformed point in global frame
 */
Eigen::Vector3d pointLocalToGlobal(const Eigen::Vector3d& local_point,
                                   const RobotPose& robot_pose);

// -----------------------------------------------------------------------------
// Kinematics helpers
// -----------------------------------------------------------------------------

/**
 * @brief Linear velocity at an arbitrary point on the tool given the end-effector twist.
 *
 * @param global_point Point whose velocity we desire (world frame)
 * @param ee_frame_pos End-effector origin position (world frame)
 * @param ee_frame_vel End-effector linear velocity (world frame)
 * @param ee_frame_omega End-effector angular velocity (world frame)
 * @return Eigen::Vector3d Velocity of the point in global frame
 */
Eigen::Vector3d getVelAt(const Eigen::Vector3d& global_point,
                         const Eigen::Vector3d& ee_frame_pos,
                         const Eigen::Vector3d& ee_frame_vel,
                         const Eigen::Vector3d& ee_frame_omega);

/**
 * @brief Linear acceleration at an arbitrary point on the tool.
 *
 * Uses the kinematic relation \( a_p = a_o + \dot{\omega}\times r + \omega\times(\omega\times r) \)
 * where *o* is the end-effector origin and *p* the queried point.
 *
 * @param global_point Point whose acceleration we desire (world frame)
 * @param ee_frame_pos End-effector origin position (world frame)
 * @param ee_frame_omega End-effector angular velocity (world frame)
 * @param ee_frame_acc End-effector linear acceleration (world frame)
 * @param ee_frame_omega_acc End-effector angular acceleration (world frame)
 * @return Eigen::Vector3d Acceleration of the point in global frame
 */
Eigen::Vector3d getAccAt(const Eigen::Vector3d& global_point,
                         const Eigen::Vector3d& ee_frame_pos,
                         const Eigen::Vector3d& ee_frame_omega,
                         const Eigen::Vector3d& ee_frame_acc,
                         const Eigen::Vector3d& ee_frame_omega_acc);

// -----------------------------------------------------------------------------
// RCM / RPY velocity computation helpers
// -----------------------------------------------------------------------------

/**
 * @brief Velocity decomposition for point-based Remote Center of Motion (RCM) control.
 *
 * Computes the linear (v) and angular (ω) velocities that move the tool tip
 * towards a target point while keeping the pivot fixed at @p global_rcm_point and
 * rolling the shaft at @p desired_roll_vel.
 *
 * @param ee_frame_pos Position of the end-effector frame
 * @param global_rcm_point Position of the RCM pivot point in global frame
 * @param global_point Position of the instrument tip in global frame
 * @param desired_point_vel Desired velocity of the instrument tip
 * @param desired_roll_vel Desired roll velocity around the shaft axis
 * @return std::pair<Eigen::Vector3d, Eigen::Vector3d> Linear and angular velocities
 */
std::pair<Eigen::Vector3d, Eigen::Vector3d> computeRcmVelocities(
    const Eigen::Vector3d& ee_frame_pos,
    const Eigen::Vector3d& global_rcm_point,
    const Eigen::Vector3d& global_point,
    const Eigen::Vector3d& desired_point_vel,
    double desired_roll_vel);

/**
 * @brief Velocity decomposition for Roll-Pitch-Yaw (RPY) translation control.
 *
 * Given desired roll/pitch/yaw rates (rad/s) and translation speed (m/s)
 * compute the corresponding end-effector linear/angular velocity pair.
 *
 * @param ee_frame_pos Position of the end-effector frame
 * @param global_rcm_point Position of the RCM pivot point in global frame
 * @param roll Desired roll velocity (around instrument axis)
 * @param pitch Desired pitch velocity
 * @param yaw Desired yaw velocity
 * @param translation Desired translation velocity along the instrument axis
 * @return std::pair<Eigen::Vector3d, Eigen::Vector3d> Linear and angular velocities
 */
std::pair<Eigen::Vector3d, Eigen::Vector3d> computeRcmVelocities(
    const Eigen::Vector3d& ee_frame_pos,
    const Eigen::Vector3d& global_rcm_point,
    double roll, double pitch, double yaw,
    double translation);

/** @brief Converts a relative tip position to relative pitch, yaw, and pivot translation. */
  std::tuple<double, double, double> convertRelativeTipPositionToPYT(
    const Eigen::Vector3d& global_pivot_position,
    const Eigen::Vector3d& initial_global_tip_position,
    const Eigen::Vector3d& relative_tip_position);


/**
 * @brief Compute the signed roll angle (spin about the instrument shaft).
 *
 * @param current_end_effector_quaternion  Current orientation of the end-effector in the base frame at this update.
 * @param initial_end_effector_quaternion  Initial orientation of the end-effector in the base frame.
 * @param initial_shaft_axis_base          Unit vector of the instrument shaft axis at initialization, expressed in the base frame.
 * @param tip_position_base                Current tip position, in base frame coordinates (metres).
 * @param pivot_position_base              Remote-centre-of-motion (pivot) position, in base frame coordinates (metres).
 * @param last_valid_roll                  Previously published roll angle (radians); returned unchanged if the shaft axis is ill-defined.
 *
 * @return Signed roll angle in radians in the interval (-π, π].
 */
double computeRoll(const Eigen::Quaterniond& current_end_effector_quaternion,
                   const Eigen::Quaterniond& initial_end_effector_quaternion,
                   const Eigen::Vector3d&    initial_shaft_axis_base,
                   const Eigen::Vector3d&    tip_position_base,
                   const Eigen::Vector3d&    pivot_position_base,
                   double                    last_valid_roll);

}