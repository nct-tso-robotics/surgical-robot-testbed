from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        "namespace": "robot1",
        "device_path": "",
        "device_name": "HID 1a86:e026 Keyboard",
        "state_topic": "foot_switch/state",
        "publish_timer_period": "0.01",
    }
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in defaults.items()
            ],
            Node(
                package="foot_switch",
                namespace=LaunchConfiguration("namespace"),
                executable="foot_switch",
                parameters=[
                    {
                        name: ParameterValue(
                            LaunchConfiguration(name),
                            value_type=float if name == "publish_timer_period" else str,
                        )
                        for name in defaults
                        if name != "namespace"
                    }
                ],
                output="screen",
            ),
        ]
    )
