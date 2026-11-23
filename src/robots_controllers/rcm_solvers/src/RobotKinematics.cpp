#include <stdexcept>
#include <algorithm>
#include <cmath>
#include "RobotKinematics.h"

namespace rcm_solvers {

    // ============================================================================
    // RobotKinematics Base Class
    // ============================================================================

    RobotKinematics::RobotKinematics(int num_joints) : joint_state_tracker_(num_joints) {}

    JointStateTracker& RobotKinematics::getJointStateTracker() {
        return joint_state_tracker_;
    }

    // Static utility functions for backward compatibility
    Eigen::Matrix4d RobotKinematics::getDHTransform(double a, double alpha, double d, double theta, bool modified_dh) {
        KDL::Frame frame = modified_dh ?
            KDL::Frame::DH_Craig1989(a, alpha, d, theta) :
            KDL::Frame::DH(a, alpha, d, theta);

        Eigen::Matrix4d T;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T(i, j) = frame.M(i, j);
            }
            T(i, 3) = frame.p(i);
        }
        T.row(3) << 0.0, 0.0, 0.0, 1.0;

        return T;
    }

    std::vector<Eigen::Matrix4d> RobotKinematics::getJointsDHTransforms(
        const std::vector<std::tuple<double, double, double, double>>& dh_params,
        bool modified_dh) {

        std::vector<Eigen::Matrix4d> transforms;
        transforms.reserve(dh_params.size());

        for (const auto& param : dh_params) {
            double a, alpha, d, theta;
            std::tie(a, alpha, d, theta) = param;
            transforms.push_back(getDHTransform(a, alpha, d, theta, modified_dh));
        }
        return transforms;
    }

    Eigen::Matrix4d RobotKinematics::calculateFK(
        const std::vector<std::tuple<double, double, double, double>>& dh_params,
        const std::vector<double>& joint_angles,
        bool modified_dh) {

        std::size_t dof = joint_angles.size();
        KDL::Chain chain;

        for (std::size_t i = 0; i < dh_params.size(); ++i) {
            double a, alpha, d, theta;
            std::tie(a, alpha, d, theta) = dh_params[i];

            KDL::Frame frame = modified_dh ?
                KDL::Frame::DH_Craig1989(a, alpha, d, i < dof ? 0.0 : theta) :
                KDL::Frame::DH(a, alpha, d, i < dof ? 0.0 : theta);

            KDL::Joint joint = (i < dof) ?
                KDL::Joint(KDL::Joint::RotZ) :
                KDL::Joint(KDL::Joint::None);

            chain.addSegment(KDL::Segment(joint, frame));
        }

        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        KDL::JntArray q(dof);
        for (std::size_t i = 0; i < dof; ++i) {
            q(i) = joint_angles[i];
        }

        KDL::Frame ee_frame;
        fk_solver.JntToCart(q, ee_frame);

        Eigen::Matrix4d T;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T(i, j) = ee_frame.M(i, j);
            }
            T(i, 3) = ee_frame.p(i);
        }
        T.row(3) << 0.0, 0.0, 0.0, 1.0;

        return T;
    }

    Eigen::MatrixXd RobotKinematics::getJacobian(
        const std::vector<std::tuple<double, double, double, double>>& dh_params,
        const std::vector<double>& joint_angles,
        bool modified_dh) {

        std::size_t dof = joint_angles.size();
        KDL::Chain chain;

        for (std::size_t i = 0; i < dh_params.size(); ++i) {
            double a, alpha, d, theta;
            std::tie(a, alpha, d, theta) = dh_params[i];

            KDL::Frame frame = modified_dh ?
                KDL::Frame::DH_Craig1989(a, alpha, d, i < dof ? 0.0 : theta) :
                KDL::Frame::DH(a, alpha, d, i < dof ? 0.0 : theta);

            KDL::Joint joint = (i < dof) ?
                KDL::Joint(KDL::Joint::RotZ) :
                KDL::Joint(KDL::Joint::None);

            chain.addSegment(KDL::Segment(joint, frame));
        }

        KDL::ChainJntToJacSolver jac_solver(chain);
        KDL::JntArray q(dof);
        for (std::size_t i = 0; i < dof; ++i) {
            q(i) = joint_angles[i];
        }

        KDL::Jacobian jac(dof);
        jac_solver.JntToJac(q, jac);

        Eigen::MatrixXd J(6, dof);
        for (std::size_t i = 0; i < 6; ++i) {
            for (std::size_t j = 0; j < dof; ++j) {
                J(i, j) = jac(i, j);
            }
        }

        return J;
    }

    Eigen::MatrixXd RobotKinematics::computeJacobianPseudoInverse(const Eigen::MatrixXd& jacobian) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const double tolerance = 1e-6;
        Eigen::VectorXd singular_values_inv = svd.singularValues();
        for (int i = 0; i < singular_values_inv.size(); ++i) {
            singular_values_inv(i) = (singular_values_inv(i) > tolerance)
                ? 1.0 / singular_values_inv(i)
                : 0.0;
        }
        return svd.matrixV() * singular_values_inv.asDiagonal() * svd.matrixU().transpose();
    }

    // ============================================================================
    // UR5eKinematics
    // ============================================================================

    UR5eKinematics::UR5eKinematics() : RobotKinematics(6) {
        // Build the KDL chain once in constructor
        // UR5e DH parameters (standard DH convention)
        // Format: DH(a [m], alpha [rad], d [m], theta [rad])
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0, M_PI/2.0, 0.1625, 0.0)));
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(-0.425, 0.0, 0.0, 0.0)));
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(-0.3922, 0.0, 0.0, 0.0)));
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0, M_PI/2.0, 0.1333, 0.0)));
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0, -M_PI/2.0, 0.0997, 0.0)));
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0, 0.0, 0.0996, 0.0)));

        // Create solvers once
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
        jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
        ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
    }

    UR5eKinematics::UR5eKinematics(const KDL::Chain& chain)
        : RobotKinematics(6), chain_(chain) {
        if (chain_.getNrOfJoints() != 6) throw std::invalid_argument("UR5e chain must have six joints");
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
        jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
        ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
    }

    std::vector<std::tuple<double, double, double, double>> UR5eKinematics::getDHParams(
        const std::vector<double>& joint_angles) const {
        return {
            std::make_tuple(0.0, M_PI / 2.0, 0.1625,  joint_angles[0]),
            std::make_tuple(-0.425, 0.0, 0.0,         joint_angles[1]),
            std::make_tuple(-0.3922, 0.0, 0.0,        joint_angles[2]),
            std::make_tuple(0.0, M_PI / 2.0, 0.1333,  joint_angles[3]),
            std::make_tuple(0.0, -M_PI / 2.0, 0.0997, joint_angles[4]),
            std::make_tuple(0.0, 0.0, 0.0996,         joint_angles[5])
        };
    }

    Eigen::Matrix4d UR5eKinematics::calculateFK(const std::vector<double>& joint_angles) const {
        if (joint_angles.size() != 6 || !std::all_of(joint_angles.begin(), joint_angles.end(), [](double v) { return std::isfinite(v); })) throw std::invalid_argument("Invalid joint positions");
        KDL::JntArray q(6);
        for (int i = 0; i < 6; i++) {
            q(i) = joint_angles[i];
        }

        KDL::Frame ee_frame;
        fk_solver_->JntToCart(q, ee_frame);

        Eigen::Matrix4d T;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T(i, j) = ee_frame.M(i, j);
            }
            T(i, 3) = ee_frame.p(i);
        }
        T.row(3) << 0.0, 0.0, 0.0, 1.0;

        return T;
    }

    Eigen::MatrixXd UR5eKinematics::getJacobian(const std::vector<double>& joint_angles) const {
        if (joint_angles.size() != 6 || !std::all_of(joint_angles.begin(), joint_angles.end(), [](double v) { return std::isfinite(v); })) throw std::invalid_argument("Invalid joint positions");
        KDL::JntArray q(6);
        for (int i = 0; i < 6; i++) {
            q(i) = joint_angles[i];
        }

        KDL::Jacobian jac(6);
        jac_solver_->JntToJac(q, jac);

        Eigen::MatrixXd J(6, 6);
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                J(i, j) = jac(i, j);
            }
        }

        return J;
    }

    Eigen::VectorXd UR5eKinematics::cartesianVelocitiesToJointVelocities(
        const std::vector<double>& joint_angles,
        const Eigen::VectorXd& cartesian_velocity) {
        if (joint_angles.size() != 6 || !std::all_of(joint_angles.begin(), joint_angles.end(), [](double v) { return std::isfinite(v); }) || cartesian_velocity.size() != 6 || !cartesian_velocity.allFinite()) throw std::invalid_argument("Invalid kinematics input");

        KDL::JntArray q(6);
        for (int i = 0; i < 6; i++) {
            q(i) = joint_angles[i];
        }

        KDL::Twist twist(
            KDL::Vector(cartesian_velocity(0), cartesian_velocity(1), cartesian_velocity(2)),
            KDL::Vector(cartesian_velocity(3), cartesian_velocity(4), cartesian_velocity(5))
        );

        KDL::JntArray qdot(6);
        if (ik_vel_solver_->CartToJnt(q, twist, qdot) < 0) throw std::runtime_error("KDL inverse velocity failed");

        Eigen::VectorXd joint_velocities(6);
        for (int i = 0; i < 6; i++) {
            joint_velocities(i) = qdot(i);
        }

        return joint_velocities;
    }

    void UR5eKinematics::setJointLimits(const std::vector<double>& lower,
        const std::vector<double>& upper, const std::vector<double>& velocity) {
        if (lower.size()!=6 || upper.size()!=6 || velocity.size()!=6)
            throw std::invalid_argument("UR5e limits must contain six joints");
        for (size_t i=0;i<6;++i)
            if (std::isnan(lower[i]) || std::isnan(upper[i]) || lower[i]>=upper[i] ||
                !std::isfinite(velocity[i]) || velocity[i]<=0)
                throw std::invalid_argument("Invalid joint limits");
        lower_limits_=lower; upper_limits_=upper; velocity_limits_=velocity;
    }

    int UR5eKinematics::getNumJoints() const {
        return 6;
    }

    std::pair<std::vector<double>, std::vector<double>> UR5eKinematics::getJointsPositionLimits() const {
        return {lower_limits_, upper_limits_};
    }

    std::vector<double> UR5eKinematics::getJointsVelocityLimits() const {
        return velocity_limits_;
    }

    std::vector<double> UR5eKinematics::getJointsAccelerationLimits() const {
        return {150, 150, 150, 150, 150, 150};
    }

    std::vector<double> UR5eKinematics::getJointsJerkLimits() const {
        return {7500, 3750, 5000, 6250, 7500, 10000};
    }

    // ============================================================================
    // FrankaPandaKinematics
    // ============================================================================

#if TESTBED_WITH_FRANKA
    FrankaPandaKinematics::FrankaPandaKinematics() : RobotKinematics(7) {
        // Build the KDL chain once in constructor
        // Franka Panda DH parameters (modified DH convention - Craig 1989)
        // Format: DH_Craig1989(a [m], alpha [rad], d [m], theta [rad])
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH_Craig1989(0.0, 0.0, 0.333, 0.0)));
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH_Craig1989(0.0, -M_PI/2.0, 0.0, 0.0)));
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH_Craig1989(0.0, M_PI/2.0, 0.316, 0.0)));
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH_Craig1989(0.0825, M_PI/2.0, 0.0, 0.0)));
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH_Craig1989(-0.0825, -M_PI/2.0, 0.384, 0.0)));
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH_Craig1989(0.0, M_PI/2.0, 0.0, 0.0)));
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH_Craig1989(0.088, M_PI/2.0, 0.0, 0.0)));

        // Fixed segments (flange and TCP)
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH_Craig1989(0.0, 0.0, 0.107, 0.0)));
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH_Craig1989(0.0, 0.0, 0.0, -M_PI/4.0)));
        chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH_Craig1989(0.0, 0.0, 0.1034, 0.0)));

        // Create solvers once
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
        jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
        ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
    }

    std::vector<std::tuple<double, double, double, double>> FrankaPandaKinematics::getDHParams(
        const std::vector<double>& joint_angles) const {
        return {
            std::make_tuple(0.0, 0, 0.333,  joint_angles[0]),
            std::make_tuple(0, -M_PI/2.0, 0.0,         joint_angles[1]),
            std::make_tuple(0, M_PI/2.0, 0.316,        joint_angles[2]),
            std::make_tuple(0.0825, M_PI/2.0, 0,  joint_angles[3]),
            std::make_tuple(-0.0825, -M_PI / 2.0, 0.384, joint_angles[4]),
            std::make_tuple(0.0, M_PI / 2.0, 0,         joint_angles[5]),
            std::make_tuple(0.088, M_PI / 2.0, 0,         joint_angles[6]),
            std::make_tuple(0, 0, 0.107, 0),
            std::make_tuple(0, 0, 0, -M_PI / 4),
            std::make_tuple(0, 0, 0.1034, 0)
        };
    }

    Eigen::Matrix4d FrankaPandaKinematics::calculateFK(const std::vector<double>& joint_angles) const {
        if (joint_angles.size() != 7 || !std::all_of(joint_angles.begin(), joint_angles.end(), [](double v) { return std::isfinite(v); })) throw std::invalid_argument("Invalid joint positions");
        KDL::JntArray q(7);
        for (int i = 0; i < 7; i++) {
            q(i) = joint_angles[i];
        }

        KDL::Frame ee_frame;
        fk_solver_->JntToCart(q, ee_frame);

        Eigen::Matrix4d T;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T(i, j) = ee_frame.M(i, j);
            }
            T(i, 3) = ee_frame.p(i);
        }
        T.row(3) << 0.0, 0.0, 0.0, 1.0;

        return T;
    }

    Eigen::MatrixXd FrankaPandaKinematics::getJacobian(const std::vector<double>& joint_angles) const {
        if (joint_angles.size() != 7 || !std::all_of(joint_angles.begin(), joint_angles.end(), [](double v) { return std::isfinite(v); })) throw std::invalid_argument("Invalid joint positions");
        KDL::JntArray q(7);
        for (int i = 0; i < 7; i++) {
            q(i) = joint_angles[i];
        }

        KDL::Jacobian jac(7);
        jac_solver_->JntToJac(q, jac);

        Eigen::MatrixXd J(6, 7);
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 7; j++) {
                J(i, j) = jac(i, j);
            }
        }

        return J;
    }

    Eigen::VectorXd FrankaPandaKinematics::cartesianVelocitiesToJointVelocities(
        const std::vector<double>& joint_angles,
        const Eigen::VectorXd& cartesian_velocity) {
        if (joint_angles.size() != 7 || !std::all_of(joint_angles.begin(), joint_angles.end(), [](double v) { return std::isfinite(v); }) || cartesian_velocity.size() != 6 || !cartesian_velocity.allFinite()) throw std::invalid_argument("Invalid kinematics input");

        KDL::JntArray q(7);
        for (int i = 0; i < 7; i++) {
            q(i) = joint_angles[i];
        }

        KDL::Twist twist(
            KDL::Vector(cartesian_velocity(0), cartesian_velocity(1), cartesian_velocity(2)),
            KDL::Vector(cartesian_velocity(3), cartesian_velocity(4), cartesian_velocity(5))
        );

        KDL::JntArray qdot(7);
        if (ik_vel_solver_->CartToJnt(q, twist, qdot) < 0) throw std::runtime_error("KDL inverse velocity failed");

        Eigen::VectorXd joint_velocities(7);
        for (int i = 0; i < 7; i++) {
            joint_velocities(i) = qdot(i);
        }

        return joint_velocities;
    }

    int FrankaPandaKinematics::getNumJoints() const {
        return 7;
    }

    std::pair<std::vector<double>, std::vector<double>> FrankaPandaKinematics::getJointsPositionLimits() const {
        std::vector<double> min_limits = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
        std::vector<double> max_limits = { 2.8973,  1.7628,  2.8973,  -0.0698,  2.8973,  3.7525,  2.8973};
        return {min_limits, max_limits};
    }

    std::vector<double> FrankaPandaKinematics::getJointsVelocityLimits() const {
        return {2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};
    }

    std::vector<double> FrankaPandaKinematics::getJointsAccelerationLimits() const {
        return {15, 7.5, 10, 12.5, 15, 20, 20};
    }

    std::vector<double> FrankaPandaKinematics::getJointsJerkLimits() const {
        return {7500, 3750, 5000, 6250, 7500, 10000, 10000};
    }

#endif
} // namespace rcm_solvers