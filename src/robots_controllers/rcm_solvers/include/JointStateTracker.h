#pragma once

#include <vector>
#include <deque>
#include <Eigen/Dense>

namespace rcm_solvers {

class JointStateTracker {
public:

    explicit JointStateTracker(size_t num_joints, size_t window_size = 5);


    void update(const std::vector<double>& joint_positions,
                const std::vector<double>& joint_velocities,
                double dt);

    void reset();

    // Getters for history
    const std::deque<std::vector<double>>& getPositionHistory() const { return position_history_; }
    const std::deque<std::vector<double>>& getVelocityHistory() const { return velocity_history_; }
    const std::deque<std::vector<double>>& getAccelerationHistory() const { return acceleration_history_; }
    const std::deque<std::vector<double>>& getJerkHistory() const { return jerk_history_; }

private:
    size_t num_joints_;
    size_t window_size_;

    std::deque<std::vector<double>> position_history_;
    std::deque<std::vector<double>> velocity_history_;
    std::deque<std::vector<double>> acceleration_history_;
    std::deque<std::vector<double>> jerk_history_;

    // Helper methods for derivative computation
    std::vector<double> computeAccelerations(const std::deque<std::vector<double>>& velocities,
                                           double dt);
    std::vector<double> computeJerks(const std::deque<std::vector<double>>& accelerations,
                                    double dt);
};

} // namespace rcm_solvers