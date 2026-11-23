#pragma once

#include "VelocityProfilers.h"
#include "Utils.h"
#include <memory>
#include "TwistSolver.h"
#include "RobotKinematics.h"
#include "VelocitySolver.h"

namespace rcm_solvers {


class JointSolver : public VelocitySolver {
public:
    explicit JointSolver(std::unique_ptr<RobotKinematics> robot_kinematics, bool log_info = false);
    void setLogInfo(bool enable) override;
    void stop() override;
    void setAngularLimits(double velocity, double acceleration, double jerk) override;

    void starting(const RobotPose& current_pose) override;

    void updateTargetPose(const Eigen::Vector3d& delta_position,
                          double delta_roll,
                          double movement_time) override;

    Eigen::VectorXd update(const RobotPose& current_pose,
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

    TwistSolver twist_solver_;
};

} // namespace rcm_solvers