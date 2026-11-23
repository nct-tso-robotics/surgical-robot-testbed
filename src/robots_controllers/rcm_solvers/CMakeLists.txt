cmake_minimum_required(VERSION 3.16)
project(rcm_solvers VERSION 0.2.0 LANGUAGES CXX)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
option(TESTBED_WITH_FRANKA "Include optional Franka Panda kinematics (no ROS hardware driver)" OFF)
option(BUILD_PYTHON_BINDINGS "Build and install pyrcm_solvers" OFF)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
add_library(rcm_solvers SHARED
  src/Utils.cpp src/TwistSolver.cpp src/VelocityProfilers.cpp
  src/RobotKinematics.cpp src/JointSolver.cpp src/JointStateTracker.cpp
  src/VelocitySolverFactory.cpp src/RobotKinematicsFactory.cpp)
add_library(rcm_solvers::rcm_solvers ALIAS rcm_solvers)
target_compile_features(rcm_solvers PUBLIC cxx_std_17)
target_compile_definitions(rcm_solvers PUBLIC TESTBED_WITH_FRANKA=$<BOOL:${TESTBED_WITH_FRANKA}>)
set_target_properties(rcm_solvers PROPERTIES CXX_EXTENSIONS OFF VERSION ${PROJECT_VERSION} SOVERSION 0)
target_include_directories(rcm_solvers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(rcm_solvers PUBLIC Eigen3::Eigen orocos-kdl)
install(TARGETS rcm_solvers EXPORT rcm_solversTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rcm_solvers)
install(EXPORT rcm_solversTargets NAMESPACE rcm_solvers:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rcm_solvers)
configure_package_config_file(cmake/rcm_solversConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/rcm_solversConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rcm_solvers
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/rcm_solversConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/rcm_solversConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/rcm_solversConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rcm_solvers)
if(BUILD_PYTHON_BINDINGS)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
  set(PYBIND11_FINDPYTHON ON)
  find_package(pybind11 CONFIG REQUIRED)
  pybind11_add_module(pyrcm_solvers src/PythonBindings.cpp)
  target_link_libraries(pyrcm_solvers PRIVATE rcm_solvers)
  set_target_properties(pyrcm_solvers PROPERTIES INSTALL_RPATH "$ORIGIN/../..")
  install(TARGETS pyrcm_solvers LIBRARY DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages)
endif()
