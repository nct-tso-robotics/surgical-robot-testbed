#include "JointSolver.h"
#include <cmath>

namespace rcm_solvers {

/**
 * @brief Construct a Joint Solver
 *
 * Creates a solver that controls the tip position of a surgical instrument while maintaining
 * a remote center of motion constraint. Controls the robot in joint space.
 *
 * @param log_info Whether to output debug information during execution
 * @param robot_tools Contains useful functions related to specific robot
 */
JointSolver::JointSolver(std::unique_ptr<RobotKinematics> robot_kinematics, bool log_info)
    : twist_solver_(std::move(robot_kinematics), log_info) {
}

void JointSolver::starting(const RobotPose& current_pose) {
    this->twist_solver_.starting(current_pose);
}


void JointSolver::updateTargetPose(
    const Eigen::Vector3d& delta_position,
    double delta_roll,
    double movement_time) {
    this->twist_solver_.updateTargetPose(delta_position, delta_roll, movement_time);
}

Eigen::VectorXd JointSolver::update(
    const RobotPose& current_pose,
    std::vector<double> joint_positions,
    std::vector<double> joint_velocities,
    double current_time,
    double dt) {
    Eigen::VectorXd cartesian_velocities = this->twist_solver_.update(current_pose, joint_positions, joint_velocities, current_time, dt);
    Eigen::VectorXd next_velocities = this->twist_solver_.getRobotKinematics().cartesianVelocitiesToJointVelocities(joint_positions, cartesian_velocities);


    return next_velocities;
}


void JointSolver::setLogInfo(bool enable) {
    this->twist_solver_.setLogInfo(enable);
}

void JointSolver::setLocalPivotPoint(const Eigen::Vector3d& pivot) {
    this->twist_solver_.setLocalPivotPoint(pivot);
}


void JointSolver::setLocalTipPoint(const Eigen::Vector3d& tip) {
    this->twist_solver_.setLocalTipPoint(tip);
}

void JointSolver::setCartesianJerkLimit(double jerk_limit) {
    this->twist_solver_.setCartesianJerkLimit(jerk_limit);
}

void JointSolver::setCartesianAccelerationLimit(double acceleration_limit) {
    this->twist_solver_.setCartesianAccelerationLimit(acceleration_limit);
}

void JointSolver::setCartesianVelocityLimit(double velocity_limit) {
    this->twist_solver_.setCartesianVelocityLimit(velocity_limit);
}


RobotKinematics& JointSolver::getRobotKinematics() {
    return this->twist_solver_.getRobotKinematics();
}

void JointSolver::stop() { twist_solver_.stop(); }
void JointSolver::setAngularLimits(double velocity, double acceleration, double jerk) {
    twist_solver_.setAngularLimits(velocity, acceleration, jerk);
}
} // namespace rcm_solvers