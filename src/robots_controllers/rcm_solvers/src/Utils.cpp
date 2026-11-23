#include <algorithm>
#include "Utils.h"
#include <Eigen/Dense>
#include <array>

namespace rcm_solvers {

/**
 * @brief Creates a RobotPose from a 4x4 transformation matrix stored as a flat array
 *
 * @param robot_transform 4x4 transformation matrix as a 16-element array in column-major order (Eigen default)
 * @return RobotPose Resulting pose containing position and orientation
 */
RobotPose getRobotPose(const std::array<double,16>& robot_transform) {
    Eigen::Matrix3d rotation;
    rotation(0) = robot_transform[0];
    rotation(1) = robot_transform[1];
    rotation(2) = robot_transform[2];
    rotation(3) = robot_transform[4];
    rotation(4) = robot_transform[5];
    rotation(5) = robot_transform[6];
    rotation(6) = robot_transform[8];
    rotation(7) = robot_transform[9];
    rotation(8) = robot_transform[10];
    Eigen::Vector3d translation(robot_transform[12], robot_transform[13], robot_transform[14]);
    RobotPose result;
    result.orientation = Eigen::Quaterniond(rotation);
    result.position    = translation;
    return result;
}

/**
 * @brief Creates a RobotPose from separate orientation quaternion and position vector
 *
 * @param orientation Quaternion representing the orientation
 * @param position Vector representing the position
 * @return RobotPose Combined pose
 */
RobotPose getRobotPose(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position) {
    RobotPose result;
    result.orientation = orientation;
    result.position    = position;
    return result;
}

/**
 * @brief Transforms a point from local (robot end-effector) frame to global (world) frame
 *
 * @param local_point Point expressed in the local frame
 * @param robot_pose Robot pose in the global frame
 * @return Eigen::Vector3d Transformed point in global frame
 */
Eigen::Vector3d pointLocalToGlobal(const Eigen::Vector3d& local_point, const RobotPose& robot_pose) {
    return robot_pose.orientation * local_point + robot_pose.position;
}

/**
 * @brief Computes the velocity of a point attached to a moving rigid body
 *
 * @param global_point Global coordinates of the point
 * @param ee_frame_pos Position of the end-effector frame origin
 * @param ee_frame_vel Linear velocity of the end-effector
 * @param ee_frame_omega Angular velocity of the end-effector
 * @return Eigen::Vector3d Velocity of the point in global frame
 */
Eigen::Vector3d getVelAt(const Eigen::Vector3d& global_point,
                         const Eigen::Vector3d& ee_frame_pos,
                         const Eigen::Vector3d& ee_frame_vel,
                         const Eigen::Vector3d& ee_frame_omega) {
    return ee_frame_vel + ee_frame_omega.cross(global_point - ee_frame_pos);
}

/**
 * @brief Computes the acceleration of a point attached to a moving rigid body
 *
 * @param global_point Global coordinates of the point
 * @param ee_frame_pos Position of the end-effector frame origin
 * @param ee_frame_omega Angular velocity of the end-effector
 * @param ee_frame_acc Linear acceleration of the end-effector
 * @param ee_frame_omega_acc Angular acceleration of the end-effector
 * @return Eigen::Vector3d Acceleration of the point in global frame
 */
Eigen::Vector3d getAccAt(const Eigen::Vector3d& global_point,
                         const Eigen::Vector3d& ee_frame_pos,
                         const Eigen::Vector3d& ee_frame_omega,
                         const Eigen::Vector3d& ee_frame_acc,
                         const Eigen::Vector3d& ee_frame_omega_acc) {
    return ee_frame_acc + ee_frame_omega_acc.cross(global_point - ee_frame_pos)
         + ee_frame_omega.cross(ee_frame_omega.cross(global_point - ee_frame_pos));
}

/**
 * @brief Computes linear and angular velocities for a tip-position RCM constraint
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
    double desired_roll_vel) {
    Eigen::Vector3d r_PEE = global_rcm_point - ee_frame_pos;
    Eigen::Vector3d r_PT  = global_point - global_rcm_point;
    double r_PT_sq = r_PT.squaredNorm();

    Eigen::Vector3d ee_omega = Eigen::Vector3d::Zero();
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();

    if (r_PT_sq > 1e-6) {
        ee_omega = r_PT.cross(desired_point_vel) / r_PT_sq;
        direction = r_PT / std::sqrt(r_PT_sq);
        ee_omega += direction * desired_roll_vel;
    }

    double vel_proj = desired_point_vel.dot(direction);
    Eigen::Vector3d v_p = vel_proj * direction;
    Eigen::Vector3d ee_v = v_p - ee_omega.cross(r_PEE);

    return {ee_v, ee_omega};
}

/**
 * @brief Computes linear and angular velocities for a pivot-RPY RCM constraint
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
    double translation) {
    Eigen::Vector3d r_PEE = global_rcm_point - ee_frame_pos;
    double r_PEE_sq = r_PEE.squaredNorm();
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
    if (r_PEE_sq > 1e-6) {
        direction = r_PEE / std::sqrt(r_PEE_sq);
    }
    Eigen::Vector3d ee_omega(pitch, yaw, 0);
    ee_omega += direction * roll;
    Eigen::Vector3d ee_v = direction * translation - ee_omega.cross(r_PEE);
    return {ee_v, ee_omega};
}

/**
 * @brief Converts a relative tip position to pitch, yaw, and pivot translation.
 *
 * This function calculates the pitch, yaw, and pivot translation based on the
 * relative position of the tip with respect to the initial tip position.
 * It uses the global pivot position and the initial global tip position to
 * determine these values, which are then published as part of the robot's state.
 *
 * @param global_pivot_position The position of the pivot point in global coordinates.
 * @param initial_global_tip_position The initial position of the tip in global coordinates.
 * @param relative_tip_position The position of the tip relative to the initial tip position.
 * @return A tuple containing the pitch, yaw, and pivot translation.
 */
std::tuple<double, double, double> convertRelativeTipPositionToPYT(
    const Eigen::Vector3d& global_pivot_position,
    const Eigen::Vector3d& initial_global_tip_position,
    const Eigen::Vector3d& relative_tip_position) {
    // Compute initial shaft axis
    Eigen::Vector3d initial_shaft_axis = (initial_global_tip_position - global_pivot_position).normalized();

    // Compute initial local x-axis, handling singularity near global z-axis
    Eigen::Vector3d initial_x_local;
    if (std::abs(initial_shaft_axis.dot(Eigen::Vector3d(0, 0, 1))) < 0.9986) { // ~3° from z-axis
        initial_x_local = initial_shaft_axis.cross(Eigen::Vector3d(0, 0, 1)).normalized();
    } else {
        initial_x_local = initial_shaft_axis.cross(Eigen::Vector3d(1, 0, 0)).normalized();
    }

    // Compute initial local y-axis
    Eigen::Vector3d initial_y_local = initial_shaft_axis.cross(initial_x_local).normalized();

    // Compute desired tip position in global coordinates
    Eigen::Vector3d tip_desired_global = initial_global_tip_position + relative_tip_position;

    // Compute vector from pivot to desired tip and its length
    Eigen::Vector3d vec = tip_desired_global - global_pivot_position;
    double total_length = vec.norm();

    // Handle singular case where tip coincides with pivot
    if (total_length < 1e-6) {
        return std::make_tuple(0.0, 0.0, 0.0);
    }
    Eigen::Vector3d tilted_axis = vec / total_length;

    // Form the initial rotation matrix
    Eigen::Matrix3d R_initial;
    R_initial.col(0) = initial_x_local;
    R_initial.col(1) = initial_y_local;
    R_initial.col(2) = initial_shaft_axis;

    // Transform tilted axis to local coordinates
    Eigen::Vector3d local_tilted_axis = R_initial.transpose() * tilted_axis;

    double a = local_tilted_axis[0]; // sin(yaw)
    double b = local_tilted_axis[1]; // -sin(pitch)*cos(yaw)
    double c = local_tilted_axis[2]; // cos(pitch)*cos(yaw)

    // Canonical branch around the initial shaft. Both Euler branches reconstruct
    // the same axis; choosing by floating residual can jump by pi at neutral.
    const double yaw = std::asin(std::clamp(a, -1.0, 1.0));
    const double pitch = std::atan2(-b, c);

    // Compute pivot translation
    double initial_pivot_tip_distance = (initial_global_tip_position - global_pivot_position).norm();
    double pivot_translation = total_length - initial_pivot_tip_distance;
    if (std::abs(pivot_translation) < 1e-6) {
            pivot_translation = 0.0;
        }
    return std::make_tuple(pitch, yaw, pivot_translation);
}


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
                   double                    last_valid_roll)
{
  // Instrument axis expressed in the base frame
  Eigen::Vector3d pivot_to_tip_vector   = tip_position_base - pivot_position_base;
  double          pivot_to_tip_distance = pivot_to_tip_vector.norm();
  if (pivot_to_tip_distance < 1e-6)          // degeneracy → keep previous value
    return last_valid_roll;

  Eigen::Vector3d current_shaft_axis_base =
      pivot_to_tip_vector / pivot_to_tip_distance;          // unit direction


  Eigen::Quaterniond total_end_effector_rotation =
      current_end_effector_quaternion *
      initial_end_effector_quaternion.inverse();

    /*“Tilt-only” quaternion that rotates initial_shaft_axis_base (u)
    onto current_shaft_axis_base (v).  Formula from  https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
        q  = [ sqrt((1 + u·v) * 2) / 2 ,
               (u × v) / sqrt((1 + u·v) * 2) ]*/

  Eigen::Vector3d  cross_product_initial_current_axis = initial_shaft_axis_base.cross(current_shaft_axis_base);
  double dot_product_initial_current_axis = initial_shaft_axis_base.dot(current_shaft_axis_base);

  // Handle antiparallel singularity (shaft flipped ~180°)
  if (dot_product_initial_current_axis < -0.999999) {
      return last_valid_roll;
  }

  double alignment_quaternion_scale = std::sqrt((1.0 + dot_product_initial_current_axis) * 2.0);

  Eigen::Quaterniond shaft_tilt_rotation(
      alignment_quaternion_scale * 0.5,
      cross_product_initial_current_axis.x() / alignment_quaternion_scale,
      cross_product_initial_current_axis.y() / alignment_quaternion_scale,
      cross_product_initial_current_axis.z() / alignment_quaternion_scale);
  shaft_tilt_rotation.normalize();     // guards against round-off

  // Residual rotation = pure spin about shaft axis
  Eigen::Quaterniond shaft_spin_rotation =
      total_end_effector_rotation * shaft_tilt_rotation.conjugate(); // inverse = conjugate

  // Signed angle: project quaternion vector onto shaft axis for sign,
  // rather than using ||q_vec|| which would lose directionality.
  double roll = 2.0 * std::atan2(
                 shaft_spin_rotation.vec().dot(current_shaft_axis_base),
                 shaft_spin_rotation.w());
  // Wrap roll to the equivalent angle within +-PI of last_valid_roll to keep rotation continuous
  roll = last_valid_roll + std::remainder(roll - last_valid_roll, 2*M_PI);
  return roll;
}

} // namespace rcm_solvers