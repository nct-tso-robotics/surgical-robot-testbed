#include "VelocityProfilers.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace rcm_solvers {

// ===== Septic polynomial velocity profiler =====

SepticPolynomialVelocityProfiler::SepticPolynomialVelocityProfiler()
    : initial_time_(0.0), total_time_(0.0), coefficients_(Eigen::VectorXd::Zero(7)),
      last_velocity_(0.0), last_acceleration_(0.0), last_jerk_(0.0) {}

double SepticPolynomialVelocityProfiler::getTotalTime() const {
    return total_time_;
}

/**
 * @brief Compute velocity, acceleration and jerk at the current time
 *
 * @param current_time Current time [s]
 * @return std::tuple<double,double,double> Velocity, acceleration and jerk
 */
std::tuple<double,double,double> SepticPolynomialVelocityProfiler::step(double current_time) {
    if (current_time < initial_time_ || (current_time - initial_time_) >= total_time_ || total_time_ == 0) {
        last_velocity_ = 0;
        last_acceleration_ = 0;
        last_jerk_ = 0;
        return {0.0, 0.0, 0.0};
    }
    auto [velocity, acceleration, jerk] = getVelocityAccelerationAndJerk(current_time - initial_time_, coefficients_);
    last_velocity_ = velocity;
    last_acceleration_ = acceleration;
    last_jerk_ = jerk;
    return {velocity, acceleration, jerk};
}

std::tuple<double,double,double> SepticPolynomialVelocityProfiler::getLastStep() const {
    return {last_velocity_, last_acceleration_, last_jerk_};
}

/**
 * @brief Plan a movement without enforcing limits
 *
 * Computes and stores the septic polynomial coefficients for the given boundary conditions.
 * Call getLimitScale() after this to check if limits would be exceeded.
 *
 * @param current_time Trajectory start time [s]
 * @param total_distance Total displacement to achieve
 * @param total_time Duration of the motion [s]
 * @param initial_velocity Starting velocity
 * @param final_velocity Target ending velocity
 * @param initial_acceleration Starting acceleration
 * @param final_acceleration Target ending acceleration
 * @param initial_jerk Starting jerk
 * @param final_jerk Target ending jerk
 */
void SepticPolynomialVelocityProfiler::planMovement(
    double current_time,
    double total_distance,
    double total_time,
    double initial_velocity,
    double final_velocity,
    double initial_acceleration,
    double final_acceleration,
    double initial_jerk,
    double final_jerk) {
    for (double value : {current_time, total_distance, total_time, initial_velocity, final_velocity,
                         initial_acceleration, final_acceleration, initial_jerk, final_jerk})
        if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite trajectory parameter");
    if (total_time <= 0.0) throw std::invalid_argument("Trajectory duration must be positive");
    initial_time_ = current_time;
    total_time_ = total_time;
    coefficients_ = getSepticPolynomialCoefficients(
        total_time, total_distance,
        initial_velocity, final_velocity,
        initial_acceleration, final_acceleration,
        initial_jerk, final_jerk);
}

/**
 * @brief Compute the scale factor needed to satisfy limits
 *
 * Returns a value in (0, 1] indicating how much the distance should be scaled.
 * Must be called after planMovement().
 *
 * @param velocity_limit Maximum allowed velocity [m/s]
 * @param acceleration_limit Maximum allowed acceleration [m/s^2]
 * @param jerk_limit Maximum allowed jerk [m/s^3]
 * @return double Scale factor (0, 1]
 */
double SepticPolynomialVelocityProfiler::getLimitScale(
    double velocity_limit,
    double acceleration_limit,
    double jerk_limit) const {
    double max_velocity = calculateMaxVelocity(total_time_);
    double max_acceleration = calculateMaxAcceleration(total_time_);
    double max_jerk = calculateMaxJerk(total_time_);

    double scale = 1.0;
    if (max_velocity > velocity_limit) {
        scale = std::min(scale, velocity_limit / max_velocity);
    }
    if (max_acceleration > acceleration_limit) {
        scale = std::min(scale, acceleration_limit / max_acceleration);
    }
    if (max_jerk > jerk_limit) {
        scale = std::min(scale, jerk_limit / max_jerk);
    }
    return scale;
}

/**
 * @brief Find the maximum safe distance scale by decomposing into BC and distance contributions.
 *
 * Since polynomial coefficients are linear in distance, the velocity/acceleration/jerk at any
 * time t can be decomposed as: signal(t) = signal_bc(t) + s * signal_dist(t)
 * where signal_bc comes from boundary conditions alone (D=0) and signal_dist is the
 * difference (full - bc). We find the max s in [0,1] such that ALL signals stay within limits.
 *
 * Must be called after planMovement() with the full (unscaled) distance.
 */
double SepticPolynomialVelocityProfiler::computeMaxSafeScale(
    double total_time,
    double initial_velocity,
    double final_velocity,
    double initial_acceleration,
    double final_acceleration,
    double initial_jerk,
    double final_jerk,
    double velocity_limit,
    double acceleration_limit,
    double jerk_limit) const {

    Eigen::VectorXd c_full = coefficients_;
    Eigen::VectorXd c_bc = getSepticPolynomialCoefficients(
        total_time, 0.0,
        initial_velocity, final_velocity,
        initial_acceleration, final_acceleration,
        initial_jerk, final_jerk);
    Eigen::VectorXd c_dist = c_full - c_bc;

    double s_max = 1.0;
    constexpr int num_samples = 50;

    for (int i = 0; i <= num_samples; ++i) {
        double t = total_time * i / num_samples;
        auto [v_bc, a_bc, j_bc] = getVelocityAccelerationAndJerk(t, c_bc);
        auto [v_dist, a_dist, j_dist] = getVelocityAccelerationAndJerk(t, c_dist);

        // For each signal/limit pair, find max s such that |signal_bc + s * signal_dist| <= limit
        auto constrain = [&](double sig_bc, double sig_dist, double limit) {
            if (std::abs(sig_dist) < 1e-12) return; // no distance contribution at this point
            // We need: -limit <= sig_bc + s * sig_dist <= limit
            // Upper bound: s * sig_dist <= limit - sig_bc
            // Lower bound: s * sig_dist >= -limit - sig_bc
            double bound_upper = (limit - sig_bc) / sig_dist;
            double bound_lower = (-limit - sig_bc) / sig_dist;
            // s must be >= 0, so we want the tightest upper bound on s
            // When sig_dist > 0: s <= bound_upper and s >= bound_lower
            // When sig_dist < 0: s >= bound_upper and s <= bound_lower (inequalities flip)
            double s_upper = (sig_dist > 0) ? bound_upper : bound_lower;
            if (s_upper >= 0) {
                s_max = std::min(s_max, s_upper);
            } else {
                // BC alone already exceeds the limit in the same direction as distance — block movement
                s_max = 0.0;
            }
        };

        constrain(v_bc, v_dist, velocity_limit);
        constrain(a_bc, a_dist, acceleration_limit);
        constrain(j_bc, j_dist, jerk_limit);
    }

    return std::max(0.0, s_max);
}

/**
 * @brief Plan movement and enforce limits by reducing magnitude if needed
 *
 * If the trajectory would exceed any limit, the movement distance is scaled down
 * while keeping the same movement time.
 *
 * @param current_time Trajectory start time [s]
 * @param total_distance Total displacement to achieve
 * @param total_time Duration of the motion [s]
 * @param initial_velocity Starting velocity
 * @param final_velocity Target ending velocity
 * @param initial_acceleration Starting acceleration
 * @param final_acceleration Target ending acceleration
 * @param initial_jerk Starting jerk
 * @param final_jerk Target ending jerk
 * @param velocity_limit Maximum allowed velocity [m/s]
 * @param acceleration_limit Maximum allowed acceleration [m/s^2]
 * @param jerk_limit Maximum allowed jerk [m/s^3]
 * @return double Scale factor applied to the distance (1.0 if no scaling needed)
 */
double SepticPolynomialVelocityProfiler::updateMovement(
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
    double jerk_limit) {
    planMovement(current_time, total_distance, total_time,
                 initial_velocity, final_velocity,
                 initial_acceleration, final_acceleration,
                 initial_jerk, final_jerk);

    double scale = computeMaxSafeScale(
        total_time,
        initial_velocity, final_velocity,
        initial_acceleration, final_acceleration,
        initial_jerk, final_jerk,
        velocity_limit, acceleration_limit, jerk_limit);
    if (scale < 1.0) {
        planMovement(current_time, total_distance * scale, total_time,
                     initial_velocity, final_velocity,
                     initial_acceleration, final_acceleration,
                     initial_jerk, final_jerk);
    }
    return scale;
}

double SepticPolynomialVelocityProfiler::calculateMaxVelocity(double total_time) const {
    auto velocity_function = [&](double t) {
        double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t, t6 = t5*t;
        return coefficients_[0] + 2*coefficients_[1]*t + 3*coefficients_[2]*t2 +
               4*coefficients_[3]*t3 + 5*coefficients_[4]*t4 +
               6*coefficients_[5]*t5 + 7*coefficients_[6]*t6;
    };

    double max_value = 0.0;
    constexpr int num_samples = 50;
    for (int i = 0; i <= num_samples; ++i) {
        double t = total_time * i / num_samples;
        max_value = std::max(max_value, std::abs(velocity_function(t)));
    }
    return max_value;
}

double SepticPolynomialVelocityProfiler::calculateMaxAcceleration(double total_time) const {
    auto acceleration_function = [&](double t) {
        double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
        return 2*coefficients_[1] + 6*coefficients_[2]*t +
               12*coefficients_[3]*t2 + 20*coefficients_[4]*t3 +
               30*coefficients_[5]*t4 + 42*coefficients_[6]*t5;
    };

    double max_value = 0.0;
    constexpr int num_samples = 50;
    for (int i = 0; i <= num_samples; ++i) {
        double t = total_time * i / num_samples;
        max_value = std::max(max_value, std::abs(acceleration_function(t)));
    }
    return max_value;
}

double SepticPolynomialVelocityProfiler::calculateMaxJerk(double total_time) const {
    auto jerk_function = [&](double t) {
        double t2 = t*t, t3 = t2*t, t4 = t3*t;
        return 6*coefficients_[2] + 24*coefficients_[3]*t +
               60*coefficients_[4]*t2 + 120*coefficients_[5]*t3 +
               210*coefficients_[6]*t4;
    };

    double max_value = 0.0;
    constexpr int num_samples = 50;
    for (int i = 0; i <= num_samples; ++i) {
        double t = total_time * i / num_samples;
        max_value = std::max(max_value, std::abs(jerk_function(t)));
    }
    return max_value;
}

Eigen::VectorXd SepticPolynomialVelocityProfiler::getSepticPolynomialCoefficients(
    double total_time,
    double distance,
    double initial_velocity,
    double final_velocity,
    double initial_acceleration,
    double final_acceleration,
    double initial_jerk,
    double final_jerk) const {

    double T = total_time;
    double T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T, T6 = T5*T, T7 = T6*T;

    Eigen::MatrixXd A(8,8);
    A.row(0) << 1, 0, 0, 0, 0, 0, 0, 0;
    A.row(1) << 1, T, T2, T3, T4, T5, T6, T7;
    A.row(2) << 0, 1, 0, 0, 0, 0, 0, 0;
    A.row(3) << 0, 1, 2*T, 3*T2, 4*T3, 5*T4, 6*T5, 7*T6;
    A.row(4) << 0, 0, 2, 0, 0, 0, 0, 0;
    A.row(5) << 0, 0, 2, 6*T, 12*T2, 20*T3, 30*T4, 42*T5;
    A.row(6) << 0, 0, 0, 6, 0, 0, 0, 0;
    A.row(7) << 0, 0, 0, 6, 24*T, 60*T2, 120*T3, 210*T4;

    Eigen::VectorXd B(8);
    B << 0, distance, initial_velocity, final_velocity,
         initial_acceleration, final_acceleration,
         initial_jerk, final_jerk;

    Eigen::VectorXd solution = A.colPivHouseholderQr().solve(B);
    return solution.tail(7);
}

std::tuple<double,double,double> SepticPolynomialVelocityProfiler::getVelocityAccelerationAndJerk(
    double current_time,
    const Eigen::VectorXd& c) const {
    double t = current_time;
    double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t, t6 = t5*t;

    double velocity = c[0] + 2*c[1]*t + 3*c[2]*t2 + 4*c[3]*t3 +
                      5*c[4]*t4 + 6*c[5]*t5 + 7*c[6]*t6;

    double acceleration = 2*c[1] + 6*c[2]*t + 12*c[3]*t2 +
                          20*c[4]*t3 + 30*c[5]*t4 + 42*c[6]*t5;

    double jerk = 6*c[2] + 24*c[3]*t + 60*c[4]*t2 +
                  120*c[5]*t3 + 210*c[6]*t4;

    return {velocity, acceleration, jerk};
}

// ===== Three-Dimensional profiler =====

ThreeDimensionalVelocityProfiler::ThreeDimensionalVelocityProfiler():
    last_velocities_(Eigen::Vector3d::Zero()),
    last_accelerations_(Eigen::Vector3d::Zero()),
    last_jerks_(Eigen::Vector3d::Zero()) {}

std::tuple<Eigen::Vector3d,Eigen::Vector3d,Eigen::Vector3d> ThreeDimensionalVelocityProfiler::step(double current_time) {
    auto [x_velocity, x_acceleration, x_jerk] = x_axis_.step(current_time);
    auto [y_velocity, y_acceleration, y_jerk] = y_axis_.step(current_time);
    auto [z_velocity, z_acceleration, z_jerk] = z_axis_.step(current_time);
    last_velocities_ = {x_velocity, y_velocity, z_velocity};
    last_accelerations_ = {x_acceleration, y_acceleration, z_acceleration};
    last_jerks_ = {x_jerk, y_jerk, z_jerk};
    return {last_velocities_, last_accelerations_, last_jerks_};
}

std::tuple<Eigen::Vector3d,Eigen::Vector3d,Eigen::Vector3d> ThreeDimensionalVelocityProfiler::getLastStep() const {
    return {last_velocities_, last_accelerations_, last_jerks_};
}

/**
 * @brief Update the 3D motion profile with new trajectory parameters
 *
 * Plans smooth trajectories for each coordinate axis. If any axis would exceed
 * the given limits, ALL axes are scaled down by the same factor to preserve
 * the movement direction.
 *
 * @return double Uniform scale factor applied to all axes (1.0 if no scaling needed)
 */
double ThreeDimensionalVelocityProfiler::updateMovement(
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
    double jerk_limit) {
    // Plan all three axes without limit checking
    x_axis_.planMovement(current_time, total_distance[0], total_time, initial_velocity[0], final_velocity[0], initial_acceleration[0], final_acceleration[0], initial_jerk[0], final_jerk[0]);
    y_axis_.planMovement(current_time, total_distance[1], total_time, initial_velocity[1], final_velocity[1], initial_acceleration[1], final_acceleration[1], initial_jerk[1], final_jerk[1]);
    z_axis_.planMovement(current_time, total_distance[2], total_time, initial_velocity[2], final_velocity[2], initial_acceleration[2], final_acceleration[2], initial_jerk[2], final_jerk[2]);

    // Find the most restrictive scale across all axes (uniform scaling preserves direction)
    double scale_x = x_axis_.computeMaxSafeScale(total_time, initial_velocity[0], final_velocity[0], initial_acceleration[0], final_acceleration[0], initial_jerk[0], final_jerk[0], velocity_limit, acceleration_limit, jerk_limit);
    double scale_y = y_axis_.computeMaxSafeScale(total_time, initial_velocity[1], final_velocity[1], initial_acceleration[1], final_acceleration[1], initial_jerk[1], final_jerk[1], velocity_limit, acceleration_limit, jerk_limit);
    double scale_z = z_axis_.computeMaxSafeScale(total_time, initial_velocity[2], final_velocity[2], initial_acceleration[2], final_acceleration[2], initial_jerk[2], final_jerk[2], velocity_limit, acceleration_limit, jerk_limit);
    double scale = std::min({scale_x, scale_y, scale_z});

    // Replan all axes with uniformly scaled distance if any limit was exceeded
    if (scale < 1.0) {
        x_axis_.planMovement(current_time, total_distance[0] * scale, total_time, initial_velocity[0], final_velocity[0], initial_acceleration[0], final_acceleration[0], initial_jerk[0], final_jerk[0]);
        y_axis_.planMovement(current_time, total_distance[1] * scale, total_time, initial_velocity[1], final_velocity[1], initial_acceleration[1], final_acceleration[1], initial_jerk[1], final_jerk[1]);
        z_axis_.planMovement(current_time, total_distance[2] * scale, total_time, initial_velocity[2], final_velocity[2], initial_acceleration[2], final_acceleration[2], initial_jerk[2], final_jerk[2]);
    }
    return scale;
}

} // namespace rcm_solvers
