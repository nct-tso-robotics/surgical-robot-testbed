cmake_minimum_required(VERSION 3.16)
project(testbed_joint_rcm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(realtime_tools REQUIRED)
set(CMAKE_CXX_STANDARD 17)
find_package(generate_parameter_library REQUIRED)
find_package(parameter_traits REQUIRED)
find_package(rcm_solvers REQUIRED)
find_package(Eigen3 REQUIRED)

# Add the library directory to the linker path
link_directories(${rcm_solvers_LIBRARY_DIRS})

# ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Message dependencies
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(testbed_msgs REQUIRED)

# Robot dependencies
find_package(testbed_hardware_interfaces REQUIRED)

# Generate parameter library
generate_parameter_library(joint_rcm_controller_parameters
  config/JointRcmControllerParameters.yaml
)

# Create the controller library
add_library(
  testbed_joint_rcm_controller
  SHARED
  src/JointRcmController.cpp
)

add_dependencies(testbed_joint_rcm_controller joint_rcm_controller_parameters)

target_include_directories(testbed_joint_rcm_controller
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rcm_solvers_INCLUDE_DIRS}
  ${joint_rcm_controller_parameters_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)

# Add ROS 2 dependencies
ament_target_dependencies(testbed_joint_rcm_controller
  realtime_tools
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  std_msgs
  sensor_msgs
  geometry_msgs
  testbed_msgs
  testbed_hardware_interfaces
  parameter_traits
)

# Link with rcm_solvers and parameter library
target_link_libraries(testbed_joint_rcm_controller
  ${rcm_solvers_LIBRARIES}
  joint_rcm_controller_parameters
)

# Install targets
install(TARGETS
  testbed_joint_rcm_controller
  joint_rcm_controller_parameters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install headers
install(FILES
  include/JointRcmController.hpp
  DESTINATION include
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install plugin description file
install(FILES
  testbed_joint_rcm_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(controller_interface testbed_joint_rcm_controller.xml)

ament_package()
