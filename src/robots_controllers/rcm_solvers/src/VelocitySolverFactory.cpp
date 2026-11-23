#include "VelocitySolverFactory.h"
#include "RobotKinematicsFactory.h"

namespace rcm_solvers {

SolverType string_to_solver_type(const std::string& solver_type_str) {
    std::string lower_str = solver_type_str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);

    if (lower_str == "twist") {
        return SolverType::TWIST;
    } else if (lower_str == "joint") {
        return SolverType::JOINT;
    }
    throw std::runtime_error("Invalid solver type: " + solver_type_str + ". Supported types are: twist, joint");
}

/**
 * @brief Create a concrete velocity solver instance for the specified solver type
 *
 * Factory method that instantiates the appropriate velocity solver implementation based on
 * the requested solver type. Currently supports TWIST and JOINT solver types.
 *
 * @param solver_type Enum specifying which velocity solver to create
 * @param robot_type Enum specifying which robot kinematics to create
 * @param log_info Boolean flag to enable logging of information
 * @return std::unique_ptr<VelocitySolver> Smart pointer to the newly created velocity solver
 */
std::unique_ptr<VelocitySolver> VelocitySolverFactory::create(rcm_solvers::SolverType solver_type, rcm_solvers::RobotType robot_type, bool log_info) {
    auto robot_kinematics = RobotKinematicsFactory::create(robot_type);

    switch (solver_type) {
        case rcm_solvers::SolverType::TWIST:
            return std::make_unique<TwistSolver>(std::move(robot_kinematics), log_info);
        case rcm_solvers::SolverType::JOINT:
            return std::make_unique<JointSolver>(std::move(robot_kinematics), log_info);
        default:
            throw std::runtime_error("Unknown solver type");
    }
}

} // namespace rcm_solvers