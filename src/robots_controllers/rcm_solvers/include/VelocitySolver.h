#pragma once

#include <Eigen/Core>
#include <vector>
#include <utility>
#include "Utils.h"
#include "RobotKinematics.h"

namespace rcm_solvers {

/**
 * @brief Abstract interface for RCM-constrained Cartesian velocity solvers.
 *
 * Implementations take the current robot state and output a linear + angular
 * velocity pair that advances the tool while respecting the chosen control
 * mode (Tip-Position, Pivot-RPY, …) and joint/RCM constraints.
 *
 * The solver itself is stateless between calls to `starting()` and `update()`;
 * any trajectory generators (velocity profilers) are kept inside the concrete
 * subclass.
 */
class VelocitySolver {
public:
    virtual ~VelocitySolver() = default;

    // Prepare solver with current robot state
    virtual void starting(const RobotPose& current) = 0;

    // target_position and new_roll are deltas (not absolute targets)
    virtual void updateTargetPose(const Eigen::Vector3d& delta_position,
                          double delta_roll,
                          double movement_time) = 0;

    // Compute linear and angular velocity commands
    virtual Eigen::VectorXd
    update(const RobotPose& current_pose,
           std::vector<double> current_joint_positions,
           std::vector<double> current_joint_velocities,
           double current_time,
           double dt) = 0;

    // Optional logging
    virtual void setLogInfo(bool enable) = 0;
    // Cancel motion while preserving the current reference frame and pivot.
    virtual void stop() = 0;
    virtual void setAngularLimits(double velocity, double acceleration, double jerk) = 0;

    virtual void setLocalPivotPoint(const Eigen::Vector3d& pivot) = 0;
    virtual void setLocalTipPoint(const Eigen::Vector3d& tip) = 0;

    virtual void setCartesianJerkLimit(double jerk_limit) = 0;
    virtual void setCartesianAccelerationLimit(double acceleration_limit) = 0;
    virtual void setCartesianVelocityLimit(double velocity_limit) = 0;

    virtual RobotKinematics& getRobotKinematics() = 0;
};

} // namespace rcm_solvers