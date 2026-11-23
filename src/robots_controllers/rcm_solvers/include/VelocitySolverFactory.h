#pragma once

#include <memory>
#include "VelocitySolver.h"
#include "RobotKinematics.h"
#include "TwistSolver.h"
#include "JointSolver.h"
#include "RobotKinematicsFactory.h"

namespace rcm_solvers {

/** Enum listing supported solver types. */
enum class SolverType { TWIST, JOINT };

/**
 * @brief Convert a string to SolverType enum
 * @param solver_type_str String representation of the solver type (case insensitive)
 * @return SolverType enum value
 * @throw std::runtime_error if the string doesn't match any valid solver type
 */
SolverType string_to_solver_type(const std::string& solver_type_str);

/**
 * @brief Factory that creates the correct VelocitySolver implementation based
 * on the requested SolverType.
 */
class VelocitySolverFactory {
public:
    /** @brief Create a velocity solver instance for the specified solver type */
    static std::unique_ptr<VelocitySolver> create(SolverType solver_type, RobotType robot_type, bool log_info);
};

} // namespace rcm_solvers