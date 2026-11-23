cmake_minimum_required(VERSION 3.16)
project(testbed_hardware_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
set(CMAKE_CXX_STANDARD 17)
find_package(rcm_solvers REQUIRED)
find_package(Eigen3 REQUIRED)

# Add the library directory to the linker path
link_directories(${rcm_solvers_LIBRARY_DIRS})

# ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Message dependencies
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(testbed_msgs REQUIRED)

# Create the hardware interface library
add_library(
  testbed_hardware_interfaces
  SHARED
  src/KinematicsFromDescription.cpp
  src/Ur5eHardwareInterface.cpp
  src/RobotHardwareInterfaceFactory.cpp
)

target_include_directories(testbed_hardware_interfaces
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rcm_solvers_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Link with rcm_solvers
target_link_libraries(testbed_hardware_interfaces
  ${rcm_solvers_LIBRARIES}
)

# Add ROS 2 dependencies
ament_target_dependencies(testbed_hardware_interfaces
  kdl_parser
  urdf
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
  sensor_msgs
  geometry_msgs
  testbed_msgs
)

# Export dependencies and targets
ament_export_dependencies(
  kdl_parser
  urdf
  rcm_solvers
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
  sensor_msgs
  geometry_msgs
  testbed_msgs
)
ament_export_include_directories(include)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install targets
install(
  TARGETS testbed_hardware_interfaces
  EXPORT export_testbed_hardware_interfaces
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_libraries(testbed_hardware_interfaces)
ament_export_targets(export_testbed_hardware_interfaces HAS_LIBRARY_TARGET)
ament_package()
