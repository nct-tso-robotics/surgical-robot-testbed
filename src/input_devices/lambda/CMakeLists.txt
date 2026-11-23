cmake_minimum_required(VERSION 3.16)
project(lambda LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(testbed_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

set(FORCE_DIMENSION_SDK_ROOT "" CACHE PATH "Force Dimension SDK installation")
find_path(FORCE_DIMENSION_INCLUDE_DIR drdc.h
  HINTS ${FORCE_DIMENSION_SDK_ROOT}/include REQUIRED)
find_library(FORCE_DIMENSION_DRD_LIBRARY drd
  HINTS ${FORCE_DIMENSION_SDK_ROOT}/lib ${FORCE_DIMENSION_SDK_ROOT}/lib/release/lin-x86_64-gcc REQUIRED)
find_library(FORCE_DIMENSION_DHD_LIBRARY dhd
  HINTS ${FORCE_DIMENSION_SDK_ROOT}/lib ${FORCE_DIMENSION_SDK_ROOT}/lib/release/lin-x86_64-gcc REQUIRED)
find_library(USB_LIBRARY usb-1.0 REQUIRED)

add_executable(${PROJECT_NAME} src/lambda_state_publisher.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Werror=return-type)
target_include_directories(${PROJECT_NAME} PRIVATE include ${FORCE_DIMENSION_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen Threads::Threads
  ${FORCE_DIMENSION_DRD_LIBRARY} ${FORCE_DIMENSION_DHD_LIBRARY} ${USB_LIBRARY} ${CMAKE_DL_LIBS})
ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs testbed_msgs)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
