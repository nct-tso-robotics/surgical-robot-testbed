#include "TwistSolver.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

namespace rcm_solvers {

/**
 * @brief Construct a Twist Solver
 *
 * Creates a solver that controls the tip position of a surgical instrument while maintaining
 * a remote center of motion constraint. Controls the robot in Cartesian space.
 *
 * @param log_info Whether to output debug information during execution
 * @param robot_tools Contains useful functions related to specific robot
 */
TwistSolver::TwistSolver(std::unique_ptr<RobotKinematics> robot_kinematics, bool log_info)
    : log_info_(log_info), robot_kinematics_(std::move(robot_kinematics)) {
    if (!robot_kinematics_) throw std::invalid_argument("Kinematics is required");
}

/**
 * @brief Initialize the solver with the current robot pose
 *
 * Computes the global pivot and tip points from their local coordinates and
 * initializes safety limits for the motion.
 *
 * @param current Current robot pose
 */
void TwistSolver::starting(const RobotPose& current_pose) {
    if (!current_pose.position.allFinite() || !current_pose.orientation.coeffs().allFinite() ||
        std::abs(current_pose.orientation.norm() - 1.0) > 1e-6 ||
        !local_tip_point_.allFinite() || !local_pivot_point_.allFinite() ||
        (local_tip_point_ - local_pivot_point_).norm() < 1e-6)
        throw std::invalid_argument("Finite pose and distinct tip/pivot are required");
    stop();
    robot_kinematics_->getJointStateTracker().reset();
    previous_roll_ = 0.0;
    started_ = true;
    this->initial_robot_pose = current_pose;
    // Initialize pivot and tip points in global frame.
    this->global_pivot_point_ = pointLocalToGlobal(this->local_pivot_point_, current_pose);
    this->initial_tip_point_  = pointLocalToGlobal(this->local_tip_point_, current_pose);
    // Compute the initial pivot tip distance.
    this->initial_pivot_tip_distance_ = (this->getInitialTipPoint() - this->getGlobalPivotPoint()).norm();
    // Compute initial shaft axis and local axes for handling RPY-based motion.
    this->initial_shaft_axis_ = (this->initial_tip_point_ - this->global_pivot_point_).normalized();
    this->update_movement_   = false;
    this->update_roll_       = false;
}


/**
 * @brief Update the target pose for the robot
 *
 * Sets desired tip position and roll angle for the motion.
 * Only updates values that have changed by more than epsilon to avoid unnecessary motion.
 *
 * @param delta_position Position delta to apply (in meters)
 * @param delta_roll Roll delta to apply (in radians)
 * @param movement_time Duration for completing the motion (in seconds)
 */
void TwistSolver::updateTargetPose(
    const Eigen::Vector3d& delta_position,
    double delta_roll,
    double movement_time) {
    if (!delta_position.allFinite() || !std::isfinite(delta_roll) ||
        !std::isfinite(movement_time) || movement_time <= 0.0)
        throw std::invalid_argument("Command must be finite and duration positive");
    // Latest command replaces BOTH axes, including zeros; never accumulate a queue.
    target_delta_position_ = delta_position;
    target_delta_roll_ = delta_roll;
    planned_motion_duration_ = movement_time;
    update_movement_ = update_roll_ = true;
}


/**
 * @brief Update the motion based on current state and compute velocities
 *
 * Calculates the required linear and angular velocities to move the tip toward
 * the target position while maintaining remote center of motion constraints.
 *
 * @param current_pose Current robot pose
 * @param joint_positions Current robot joint positions (in radians)
 * @param joint_velocities Current robot joint velocities (in radians/sec)
 * @param current_time Current time (in seconds)
 * @param dt Time step since last update (in seconds)
 * @return std::pair<Eigen::Vector3d, Eigen::Vector3d> Linear and angular velocities to command
 */
Eigen::VectorXd TwistSolver::update(
    const RobotPose& current_pose,
    std::vector<double> joint_positions,
    std::vector<double> joint_velocities,
    double current_time,
    double dt) {
    if (!started_) throw std::logic_error("Call starting before update");
    if (!std::isfinite(current_time) || !std::isfinite(dt) || dt <= 0.0 ||
        !current_pose.position.allFinite() || !current_pose.orientation.coeffs().allFinite())
        throw std::invalid_argument("Invalid state or time");
    this->getRobotKinematics().getJointStateTracker().update(joint_positions, joint_velocities, dt);
    Eigen::Vector3d current_tip_global = pointLocalToGlobal(this->local_tip_point_, current_pose);

    double current_roll = computeRoll(current_pose.orientation,
                          this->initial_robot_pose.orientation,
                          this->initial_shaft_axis_,
                          current_tip_global,
                          this->global_pivot_point_,
                          this->previous_roll_);
    this->previous_roll_ = current_roll;


    if (this->log_info_) {
        std::cout << "Current tip global: " << current_tip_global.transpose() << std::endl;
        std::cout << "Current roll: " << current_roll << std::endl;
    }

    // Plan translation movement using the delta directly
    if (this->update_movement_) {
        auto [last_velocity, last_acceleration, last_jerk] = this->tip_velocity_controller_.getLastStep();
        this->tip_velocity_controller_.updateMovement(
            current_time,
            this->target_delta_position_,
            this->planned_motion_duration_,
            last_velocity,
            Eigen::Vector3d::Zero(),
            last_acceleration,
            Eigen::Vector3d::Zero(),
            last_jerk,
            Eigen::Vector3d::Zero(),
            this->velocity_limit_, this->acceleration_limit_, this->jerk_limit_
        );
        this->update_movement_ = false;
    }
    // Plan roll movement using the delta directly
    if (this->update_roll_) {
        auto [last_velocity, last_acceleration, last_jerk] = this->roll_velocity_controller_.getLastStep();
        this->roll_velocity_controller_.updateMovement(
            current_time,
            this->target_delta_roll_,
            this->planned_motion_duration_,
            last_velocity,
            0.0,
            last_acceleration,
            0.0,
            last_jerk,
            0.0,
            angular_velocity_limit_, angular_acceleration_limit_, angular_jerk_limit_
        );
        this->update_roll_ = false;
    }

    // Execute one step
    auto [linear_velocity, tip_acceleration, tip_jerk] = this->tip_velocity_controller_.step(current_time);
    auto [roll_velocity, roll_acceleration, roll_jerk] = this->roll_velocity_controller_.step(current_time);

    // Compute joint frame velocities
    auto [ee_linear_velocity, ee_angular_velocity] = computeRcmVelocities(
        current_pose.position,
        this->global_pivot_point_,
        current_tip_global,
        linear_velocity,
        roll_velocity
    );
    Eigen::VectorXd velocities(6);
    velocities.head(3) = ee_linear_velocity;
    velocities.tail(3) = ee_angular_velocity;
    return velocities;
}

/**
 * @brief Enable or disable debug logging
 *
 * @param enable Set to true to enable logging, false to disable
 */
void TwistSolver::setLogInfo(bool enable) {
    this->log_info_ = enable;
}

/**
 * @brief Set the pivot point in relative coordinates
 *
 * @param pivot The pivot point coordinates in local frame (in meters)
 */
void TwistSolver::setLocalPivotPoint(const Eigen::Vector3d& pivot) {
    this->local_pivot_point_ = pivot;
}

/**
 * @brief Set the tip point in relative coordinates
 *
 * @param tip The tip point coordinates in local frame (in meters)
 */
void TwistSolver::setLocalTipPoint(const Eigen::Vector3d& tip) {
    this->local_tip_point_ = tip;
}

void TwistSolver::setCartesianJerkLimit(double jerk_limit) {
    if (!std::isfinite(jerk_limit) || jerk_limit <= 0.0) throw std::invalid_argument("Limit must be positive and finite");
    this->jerk_limit_ = jerk_limit;
}

void TwistSolver::setCartesianAccelerationLimit(double acceleration_limit) {
    if (!std::isfinite(acceleration_limit) || acceleration_limit <= 0.0) throw std::invalid_argument("Limit must be positive and finite");
    this->acceleration_limit_ = acceleration_limit;
}

void TwistSolver::setCartesianVelocityLimit(double velocity_limit) {
    if (!std::isfinite(velocity_limit) || velocity_limit <= 0.0) throw std::invalid_argument("Limit must be positive and finite");
    this->velocity_limit_ = velocity_limit;
}

/**
 * @brief Get the initial tip point in global coordinates
 *
 * @return Eigen::Vector3d Initial tip point in global frame (in meters)
 */
Eigen::Vector3d TwistSolver::getInitialTipPoint() const {
    return this->initial_tip_point_;
}

/**
 * @brief Get the pivot point in global coordinates
 *
 * @return Eigen::Vector3d Pivot point in global frame (in meters)
 */
Eigen::Vector3d TwistSolver::getGlobalPivotPoint() const {
    return this->global_pivot_point_;
}

/**
 * @brief Get the pivot point relative coordinates
 *
 * @return Eigen::Vector3d Pivot point in local frame (in meters)
 */
Eigen::Vector3d TwistSolver::getLocalPivotPoint() const {
    return this->local_pivot_point_;
}

/**
 * @brief Get the tip point relative coordinates
 *
 * @return Eigen::Vector3d Tip point in local frame (in meters)
 */
Eigen::Vector3d TwistSolver::getLocalTipPoint() const {
    return this->local_tip_point_;
}


/**
 * @brief Get the distance between the initial tip position and the initial pivot point in global frame
 *
 * @return double distance (in meters)
 */
double TwistSolver::getInitialPivotTipDistance() const  {
    return this->initial_pivot_tip_distance_;
}

RobotKinematics& TwistSolver::getRobotKinematics() {
    return *robot_kinematics_;
}

void TwistSolver::stop() {
    tip_velocity_controller_ = ThreeDimensionalVelocityProfiler();
    roll_velocity_controller_ = SepticPolynomialVelocityProfiler();
    target_delta_position_.setZero(); target_delta_roll_ = 0.0;
    update_movement_ = update_roll_ = false;
}
void TwistSolver::setAngularLimits(double velocity, double acceleration, double jerk) {
    if (!std::isfinite(velocity) || !std::isfinite(acceleration) || !std::isfinite(jerk) ||
        velocity <= 0 || acceleration <= 0 || jerk <= 0)
        throw std::invalid_argument("Angular limits must be positive and finite");
    angular_velocity_limit_ = velocity; angular_acceleration_limit_ = acceleration; angular_jerk_limit_ = jerk;
}
} // namespace rcm_solvers