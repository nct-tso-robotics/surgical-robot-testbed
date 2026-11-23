#pragma once
#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <memory>
#include "Utils.h"
#include "JointStateTracker.h"

namespace rcm_solvers {

class RobotKinematics {
public:
    virtual ~RobotKinematics() = default;
    explicit RobotKinematics(int num_joints);

    virtual std::vector<std::tuple<double, double, double, double>> getDHParams(const std::vector<double>& joint_angles) const = 0;
    virtual Eigen::Matrix4d calculateFK(const std::vector<double>& joint_angles) const = 0;
    virtual Eigen::MatrixXd getJacobian(const std::vector<double>& joint_angles) const = 0;
    virtual int getNumJoints() const = 0;
    virtual std::pair<std::vector<double>, std::vector<double>> getJointsPositionLimits() const = 0;
    virtual std::vector<double> getJointsVelocityLimits() const = 0;
    virtual std::vector<double> getJointsAccelerationLimits() const = 0;
    virtual std::vector<double> getJointsJerkLimits() const = 0;
    virtual Eigen::VectorXd cartesianVelocitiesToJointVelocities(const std::vector<double>& joint_angles, const Eigen::VectorXd& cartesian_velocity) = 0;

    JointStateTracker& getJointStateTracker();

    // Static utility functions (kept for backward compatibility)
    static Eigen::Matrix4d getDHTransform(double a, double alpha, double d, double theta, bool modified_dh);
    static std::vector<Eigen::Matrix4d> getJointsDHTransforms(const std::vector<std::tuple<double, double, double, double>>& dh_params, bool modified_dh);
    static Eigen::Matrix4d calculateFK(const std::vector<std::tuple<double, double, double, double>>& dh_params, const std::vector<double>& joint_angles, bool modified_dh);
    static Eigen::MatrixXd getJacobian(const std::vector<std::tuple<double, double, double, double>>& dh_params, const std::vector<double>& joint_angles, bool modified_dh);
    static Eigen::MatrixXd computeJacobianPseudoInverse(const Eigen::MatrixXd& jacobian);

protected:
    JointStateTracker joint_state_tracker_;
};

class UR5eKinematics : public RobotKinematics {
public:
    explicit UR5eKinematics();
    // A calibrated chain supplied by a caller, without any ROS dependency.
    explicit UR5eKinematics(const KDL::Chain& chain);
    void setJointLimits(const std::vector<double>& lower, const std::vector<double>& upper,
                        const std::vector<double>& velocity);

    std::vector<std::tuple<double, double, double, double>> getDHParams(const std::vector<double>& joint_angles) const override;
    Eigen::Matrix4d calculateFK(const std::vector<double>& joint_angles) const override;
    Eigen::MatrixXd getJacobian(const std::vector<double>& joint_angles) const override;
    int getNumJoints() const override;
    std::pair<std::vector<double>, std::vector<double>> getJointsPositionLimits() const override;
    std::vector<double> getJointsVelocityLimits() const override;
    std::vector<double> getJointsAccelerationLimits() const override;
    std::vector<double> getJointsJerkLimits() const override;
    Eigen::VectorXd cartesianVelocitiesToJointVelocities(const std::vector<double>& joint_angles, const Eigen::VectorXd& cartesian_velocity) override;

private:
    std::vector<double> lower_limits_ = std::vector<double>(6, -6.28319);
    std::vector<double> upper_limits_ = std::vector<double>(6, 6.28319);
    std::vector<double> velocity_limits_ = std::vector<double>(6, 2.175);
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
};

#if TESTBED_WITH_FRANKA
class FrankaPandaKinematics : public RobotKinematics {
public:
    explicit FrankaPandaKinematics();

    std::vector<std::tuple<double, double, double, double>> getDHParams(const std::vector<double>& joint_angles) const override;
    Eigen::Matrix4d calculateFK(const std::vector<double>& joint_angles) const override;
    Eigen::MatrixXd getJacobian(const std::vector<double>& joint_angles) const override;
    int getNumJoints() const override;
    std::pair<std::vector<double>, std::vector<double>> getJointsPositionLimits() const override;
    std::vector<double> getJointsVelocityLimits() const override;
    std::vector<double> getJointsAccelerationLimits() const override;
    std::vector<double> getJointsJerkLimits() const override;
    Eigen::VectorXd cartesianVelocitiesToJointVelocities(const std::vector<double>& joint_angles, const Eigen::VectorXd& cartesian_velocity) override;

private:
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
};

#endif
} // namespace rcm_solvers