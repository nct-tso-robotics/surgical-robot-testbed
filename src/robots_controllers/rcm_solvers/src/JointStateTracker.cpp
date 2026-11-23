#include "JointStateTracker.h"
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace rcm_solvers {

/**
 * @brief This class is used to track the joint state of a robot. Tracks the history of the joint positions, velocities, jerks, and accelerations.
 * @param num_joints Number of joints to track
 * @param window_size Size of the history window for storing joint states
 */
JointStateTracker::JointStateTracker(size_t num_joints, size_t window_size)
    : num_joints_(num_joints)
    , window_size_(window_size)
{}

/**
 * @brief Updates the joint state history with new position and velocity data
 * @param joint_positions Vector of current joint positions
 * @param joint_velocities Vector of current joint velocities
 * @param dt Time step between updates
 */
void JointStateTracker::update(const std::vector<double>& joint_positions,
                              const std::vector<double>& joint_velocities,
                              double dt)
{
    if (joint_positions.size() != num_joints_ || joint_velocities.size() != num_joints_ ||
        !std::isfinite(dt) || dt <= 0.0) throw std::invalid_argument("Invalid joint sample or period");
    this->position_history_.push_back(joint_positions);
    this->velocity_history_.push_back(joint_velocities);

    if (this->position_history_.size() > this->window_size_) {
        this->position_history_.pop_front();
    }

    if (this->velocity_history_.size() > this->window_size_) {
        this->velocity_history_.pop_front();
    }

    this->acceleration_history_.push_back(this->computeAccelerations(this->velocity_history_, dt));

    if (this->acceleration_history_.size() > this->window_size_) {
        this->acceleration_history_.pop_front();
    }

    this->jerk_history_.push_back(this->computeJerks(this->acceleration_history_, dt));

    if (this->jerk_history_.size() > this->window_size_) {
        this->jerk_history_.pop_front();
    }
}

/**
 * @brief Computes joint accelerations from velocity history
 * @param velocities Deque containing velocity history
 * @param dt Time step between velocity measurements
 * @return Vector of computed joint accelerations
 */
std::vector<double> JointStateTracker::computeAccelerations(
    const std::deque<std::vector<double>>& velocities,
    double dt)
{
    std::vector<double> accelerations(this->num_joints_, 0.0);

    // If we have at least 2 velocities, compute acceleration
    if (velocities.size() >= 2) {
        for (size_t i = 0; i < this->num_joints_; ++i) {
            double dv = velocities.back()[i] - velocities[velocities.size() - 2][i];
            accelerations[i] = dv / dt;
        }
    }

    return accelerations;
}

/**
 * @brief Computes joint jerks from acceleration history
 * @param accelerations Deque containing acceleration history
 * @param dt Time step between acceleration measurements
 * @return Vector of computed joint jerks
 */
std::vector<double> JointStateTracker::computeJerks(
    const std::deque<std::vector<double>>& accelerations,
    double dt)
{
    std::vector<double> jerks(this->num_joints_, 0.0);

    // If we have at least 2 accelerations, compute jerk
    if (accelerations.size() >= 2) {
        for (size_t i = 0; i < this->num_joints_; ++i) {
            double da = accelerations.back()[i] - accelerations[accelerations.size() - 2][i];
            jerks[i] = da / dt;
        }
    }

    return jerks;
}

void JointStateTracker::reset() {
    position_history_.clear(); velocity_history_.clear();
    acceleration_history_.clear(); jerk_history_.clear();
}
} // namespace rcm_solvers