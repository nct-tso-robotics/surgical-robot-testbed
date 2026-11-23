cmake_minimum_required(VERSION 3.16)
project(testbed_msgs)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/LambdaState.msg"
        "msg/TargetTipPose.msg"
        "msg/RobotState.msg"
        "msg/Ps4State.msg"
        "msg/PedalBoardState.msg"
        "msg/FootSwitchState.msg"
        "msg/KoalaEthercatCommand.msg"
        "msg/KoalaEthercatState.msg"
        "msg/KoalaMoveParams.msg"
        "msg/KoalaCommand.msg"
        "msg/LanguageInstruction.msg"
        "msg/ForceSensorState.msg"
        "srv/KoalaCommand.srv"
	"srv/Device.srv"
	"srv/Transformation.srv"
	"srv/GetCoordinate.srv"
        DEPENDENCIES
        std_msgs
        sensor_msgs
        geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
