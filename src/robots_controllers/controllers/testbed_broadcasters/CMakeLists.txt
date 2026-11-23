cmake_minimum_required(VERSION 3.16)
project(testbed_broadcasters)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rcm_solvers REQUIRED)

# Add the library directory to the linker path
link_directories(${rcm_solvers_LIBRARY_DIRS})

# ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

# Message dependencies
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(testbed_msgs REQUIRED)

# Robot dependencies
find_package(testbed_hardware_interfaces REQUIRED)

# Generate parameter library
generate_parameter_library(rcm_broadcaster_parameters
  config/RcmBroadcasterParameters.yaml
)

# Create the controller library
add_library(
  testbed_broadcasters
  SHARED
  src/RcmBroadcaster.cpp
)

target_include_directories(testbed_broadcasters
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rcm_solvers_INCLUDE_DIRS}
  ${rcm_broadcaster_parameters_INCLUDE_DIRS}
)

# Add ROS 2 dependencies
ament_target_dependencies(testbed_broadcasters
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  std_msgs
  sensor_msgs
  geometry_msgs
  testbed_msgs
  testbed_hardware_interfaces
)

# Link with rcm_solvers and parameter library
target_link_libraries(testbed_broadcasters
  ${rcm_solvers_LIBRARIES}
  rcm_broadcaster_parameters
)

# Install targets
install(TARGETS
  testbed_broadcasters
  rcm_broadcaster_parameters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install headers
install(FILES
  include/RcmBroadcaster.hpp
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
  testbed_broadcasters_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(controller_interface testbed_broadcasters_plugin.xml)

ament_package()
