#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "JointSolver.h"
#include "RobotKinematicsFactory.h"
#include "VelocitySolverFactory.h"
namespace py = pybind11;
using namespace rcm_solvers;
PYBIND11_MODULE(pyrcm_solvers, m) {
  m.doc() = "ROS-independent RCM kinematics and latest-command motion solvers";
  py::enum_<RobotType>(m, "RobotType").value("UR5E", RobotType::UR5E).value("FRANKA", RobotType::FRANKA);
  py::enum_<SolverType>(m, "SolverType").value("TWIST", SolverType::TWIST).value("JOINT", SolverType::JOINT);
  py::class_<Eigen::Quaterniond>(m, "Quaterniond")
    .def(py::init<double,double,double,double>(), py::arg("w"),py::arg("x"),py::arg("y"),py::arg("z"))
    .def_property_readonly("coefficients_xyzw", [](const Eigen::Quaterniond& q) { return q.coeffs().eval(); });
  py::class_<RobotPose>(m, "RobotPose")
    .def(py::init([] { return getRobotPose(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()); }))
    .def_readwrite("position", &RobotPose::position).def_readwrite("orientation", &RobotPose::orientation);
  py::class_<RobotKinematics>(m, "RobotKinematics")
    .def("calculateFK", py::overload_cast<const std::vector<double>&>(&RobotKinematics::calculateFK, py::const_))
    .def("getJacobian", py::overload_cast<const std::vector<double>&>(&RobotKinematics::getJacobian, py::const_))
    .def("getNumJoints", &RobotKinematics::getNumJoints)
    .def("cartesianVelocitiesToJointVelocities", &RobotKinematics::cartesianVelocitiesToJointVelocities);
  py::class_<UR5eKinematics, RobotKinematics>(m, "UR5eKinematics").def(py::init<>());
  m.attr("with_franka") = bool(TESTBED_WITH_FRANKA);
#if TESTBED_WITH_FRANKA
  py::class_<FrankaPandaKinematics, RobotKinematics>(m, "FrankaPandaKinematics").def(py::init<>());
#endif
  py::class_<VelocitySolver>(m, "VelocitySolver")
    .def("starting", &VelocitySolver::starting)
    .def("stop", &VelocitySolver::stop)
    .def("updateTargetPose", &VelocitySolver::updateTargetPose,
         py::arg("delta_position"), py::arg("delta_roll"), py::arg("movement_time"))
    .def("update", &VelocitySolver::update)
    .def("setLocalPivotPoint", &VelocitySolver::setLocalPivotPoint)
    .def("setLocalTipPoint", &VelocitySolver::setLocalTipPoint)
    .def("setCartesianVelocityLimit", &VelocitySolver::setCartesianVelocityLimit)
    .def("setCartesianAccelerationLimit", &VelocitySolver::setCartesianAccelerationLimit)
    .def("setCartesianJerkLimit", &VelocitySolver::setCartesianJerkLimit)
    .def("setAngularLimits", &VelocitySolver::setAngularLimits)
    .def("getRobotKinematics", &VelocitySolver::getRobotKinematics, py::return_value_policy::reference_internal);
  py::class_<TwistSolver, VelocitySolver>(m, "TwistSolver")
    .def(py::init([](RobotType type) { return std::make_unique<TwistSolver>(RobotKinematicsFactory::create(type)); }), py::arg("robot_type") = RobotType::UR5E);
  py::class_<JointSolver, VelocitySolver>(m, "JointSolver")
    .def(py::init([](RobotType type) { return std::make_unique<JointSolver>(RobotKinematicsFactory::create(type)); }), py::arg("robot_type") = RobotType::UR5E);
  m.def("create_solver", &VelocitySolverFactory::create, py::arg("solver_type"), py::arg("robot_type"), py::arg("log_info") = false);
}
