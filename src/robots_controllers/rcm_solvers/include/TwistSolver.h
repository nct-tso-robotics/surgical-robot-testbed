#pragma once

#include "VelocitySolver.h"
#include "VelocityProfilers.h"
#include <memory>
#include "Utils.h"
#include "RobotKinematics.h"
#include "JointStateTracker.h"

namespace rcm_solvers {

/**
 * @brief Solves for Cartesian velocities that move the instrument tip to a
 * desired 3-dimensional position while keeping a Remote-Center-of-Motion (pivot) fixed.
 *
 * The solver enforces:
 *   • Tip position constraint via three-dimensional tip_velocity_controller_ profiler
 *   • Shaft roll rate via one-dimensional roll_velocity_controller_
 *   • Workspace, roll, rpy and joint limits via PointInPolygon / MinMaxRoll / JointLimitMonitor
 */
class TwistSolver : public VelocitySolver {
public:
    explicit TwistSolver(std::unique_ptr<RobotKinematics> robot_kinematics, bool log_info = false);
    void setLogInfo(bool enable) override;
    void stop() override;
    void setAngularLimits(double velocity, double acceleration, double jerk) override;

    void starting(const RobotPose& current_pose) override;

/**
 * @brief Update the target (desired) pose for the robot, so that the next controller's call to the `update` function will compute the new velocities.
 */
    void updateTargetPose(const Eigen::Vector3d& delta_position,
                          double delta_roll,
                          double movement_time) override;


    /**
    * @brief Calculates the required linear and angular velocities to move the tip toward
    * the target position while maintaining remote center of motion constraints.
    */
    Eigen::VectorXd
    update(const RobotPose& current_pose,
           std::vector<double> joint_positions,
           std::vector<double> joint_velocities,
           double current_time,
           double dt) override;

    void setLocalPivotPoint(const Eigen::Vector3d& pivot) override;
    void setLocalTipPoint(const Eigen::Vector3d& tip) override;

    void setCartesianJerkLimit(double jerk_limit) override;
    void setCartesianAccelerationLimit(double acceleration_limit) override;
    void setCartesianVelocityLimit(double velocity_limit) override;

    RobotKinematics& getRobotKinematics() override;

private:
    Eigen::Vector3d getInitialTipPoint() const;
    Eigen::Vector3d getGlobalPivotPoint() const;
    Eigen::Vector3d getLocalPivotPoint() const;
    Eigen::Vector3d getLocalTipPoint() const;
    double getInitialPivotTipDistance() const;

    // Profilers
    ThreeDimensionalVelocityProfiler tip_velocity_controller_;
    SepticPolynomialVelocityProfiler roll_velocity_controller_;

    // State
    double previous_roll_{0.0};
    double initial_pivot_tip_distance_{0.0};

    // Delta targets from last input message
    Eigen::Vector3d target_delta_position_{Eigen::Vector3d::Zero()};
    double target_delta_roll_{0.0};

    Eigen::Vector3d initial_tip_point_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d global_pivot_point_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d local_pivot_point_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d local_tip_point_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d initial_shaft_axis_ = Eigen::Vector3d::Zero();
    RobotPose initial_robot_pose;

    bool update_movement_{false};
    bool update_roll_{false};

    double planned_motion_duration_{0.0};

    bool log_info_{false};

    double velocity_limit_{0.05};
    double jerk_limit_{1.0};
    double acceleration_limit_{0.25};
    double angular_velocity_limit_{0.5};
    double angular_acceleration_limit_{1.0};
    double angular_jerk_limit_{4.0};
    bool started_{false};

    std::unique_ptr<RobotKinematics> robot_kinematics_;
};

} // namespace rcm_solvers