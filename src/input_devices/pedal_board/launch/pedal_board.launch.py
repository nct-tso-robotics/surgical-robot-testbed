from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        "namespace": "robot1",
        "device_path": "",
        "device_name": "Adafruit Feather",
        "state_topic": "pedal_board/state",
        "publish_timer_period": "0.01",
    }
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in defaults.items()
            ],
            Node(
                package="pedal_board",
                namespace=LaunchConfiguration("namespace"),
                executable="pedal_board",
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
