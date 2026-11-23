#pragma once

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <tuple>
#include <utility>

namespace rcm_solvers {

/**
 * @brief 1-D septic (7th degree) polynomial velocity/acceleration/jerk profiler.
 *
 * Generates a time-continuous trajectory that meets boundary conditions in
 * position, velocity, acceleration and jerk while respecting velocity / acceleration / jerk limits.
 * When limits would be exceeded, the movement magnitude is reduced to stay within limits
 * while preserving the requested movement time.
 */
class SepticPolynomialVelocityProfiler {
public:
    SepticPolynomialVelocityProfiler();

    double getTotalTime() const;

    std::tuple<double, double, double> step(double current_time);

    // Plans the polynomial without limit checking (stores coefficients and time)
    void planMovement(
        double current_time,
        double total_distance,
        double total_time,
        double initial_velocity,
        double final_velocity,
        double initial_acceleration,
        double final_acceleration,
        double initial_jerk,
        double final_jerk);

    // Returns the scale factor (0-1] needed to satisfy limits (call after planMovement)
    double getLimitScale(
        double velocity_limit,
        double acceleration_limit,
        double jerk_limit) const;

    // Plans movement, enforces limits by reducing magnitude. Returns scale factor applied.
    double updateMovement(
        double current_time,
        double total_distance,
        double total_time,
        double initial_velocity,
        double final_velocity,
        double initial_acceleration,
        double final_acceleration,
        double initial_jerk,
        double final_jerk,
        double velocity_limit,
        double acceleration_limit,
        double jerk_limit);

    std::tuple<double, double, double> getLastStep() const;

    // Finds max distance scale s in [0,1] such that BC + s*dist contribution stays within limits.
    // Must be called after planMovement() with the full (unscaled) distance.
    double computeMaxSafeScale(
        double total_time,
        double initial_velocity,
        double final_velocity,
        double initial_acceleration,
        double final_acceleration,
        double initial_jerk,
        double final_jerk,
        double velocity_limit,
        double acceleration_limit,
        double jerk_limit) const;

private:
    double initial_time_;
    double total_time_;
    Eigen::VectorXd coefficients_;

    double last_velocity_;
    double last_acceleration_;
    double last_jerk_;

    double calculateMaxVelocity(double total_time) const;
    double calculateMaxAcceleration(double total_time) const;
    double calculateMaxJerk(double total_time) const;

    Eigen::VectorXd getSepticPolynomialCoefficients(
        double total_time,
        double distance,
        double initial_velocity,
        double final_velocity,
        double initial_acceleration,
        double final_acceleration,
        double initial_jerk,
        double final_jerk) const;

    std::tuple<double, double, double> getVelocityAccelerationAndJerk(
        double current_time,
        const Eigen::VectorXd& coefficients) const;
};

/**
 * @brief Helper that combines three independent SepticPolynomialVelocityProfiler
 * instances (X, Y, Z) to produce three-dimensional motion commands.
 * Uses uniform scaling across all axes to preserve movement direction.
 */
class ThreeDimensionalVelocityProfiler {
public:
    ThreeDimensionalVelocityProfiler();

    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> step(double current_time);

    // Returns the scale factor applied (1.0 if no scaling needed)
    double updateMovement(
        double current_time,
        const Eigen::Vector3d& total_distance,
        double total_time,
        const Eigen::Vector3d& initial_velocity,
        const Eigen::Vector3d& final_velocity,
        const Eigen::Vector3d& initial_acceleration,
        const Eigen::Vector3d& final_acceleration,
        const Eigen::Vector3d& initial_jerk,
        const Eigen::Vector3d& final_jerk,
        double velocity_limit,
        double acceleration_limit,
        double jerk_limit);

    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> getLastStep() const;

private:
    SepticPolynomialVelocityProfiler x_axis_;
    SepticPolynomialVelocityProfiler y_axis_;
    SepticPolynomialVelocityProfiler z_axis_;

    Eigen::Vector3d last_velocities_;
    Eigen::Vector3d last_accelerations_;
    Eigen::Vector3d last_jerks_;
};

} // namespace rcm_solvers
