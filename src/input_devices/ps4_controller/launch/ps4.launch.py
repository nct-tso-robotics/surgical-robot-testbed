from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        "namespace": "robot1",
        "controller_id": "0",
        "publish_rate": "30.0",
        "device_path": "",
        "device_name": "",
        "state_topic": "ps4_controller/state",
        "robot_state_topic": "robot_state",
        "robot_pose_topic": "set_tip_pose",
        "robot_base_frame": PythonExpression(
            [
                "'",
                LaunchConfiguration("namespace"),
                "'.strip('/') + '/base_link' if '",
                LaunchConfiguration("namespace"),
                "'.strip('/') else 'base_link'",
            ]
        ),
        "movement_time": "0.5",
        "input_position_scaling": "1000.0",
        "input_rotation_scaling": "100.0",
        "camera_coordinates_control": "false",
        "camera_frame": "endoscope/camera_optical_frame",
        "control_mode": "tip_pose",
        "safety_limits_xyz": "[0.05, 0.05, 0.05]",
        "safety_roll_limit": "0.2",
        "use_koala_grasp": "false",
        "gripper_transport": "service",
        "koala_command_topic": "/koala_controller/command",
        "gripper_command_topic": "/koala_grasper/open",
    }
    types = {
        "controller_id": int,
        "publish_rate": float,
        "movement_time": float,
        "input_position_scaling": float,
        "input_rotation_scaling": float,
        "camera_coordinates_control": bool,
        "use_koala_grasp": bool,
        "safety_roll_limit": float,
        "safety_limits_xyz": None,
    }

    def parameters(names):
        return {
            name: ParameterValue(
                LaunchConfiguration(name), value_type=types.get(name, str)
            )
            for name in names
        }

    device_keys = [
        "controller_id",
        "publish_rate",
        "state_topic",
        "device_path",
        "device_name",
    ]
    motion_keys = [
        name for name in defaults if name not in device_keys and name != "namespace"
    ]
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in defaults.items()
            ],
            Node(
                package="ps4_controller",
                executable="ps4_state_publisher",
                namespace=LaunchConfiguration("namespace"),
                parameters=[parameters(device_keys)],
                output="screen",
            ),
            Node(
                package="input_translation",
                executable="ps4_robot_pose_setter",
                namespace=LaunchConfiguration("namespace"),
                parameters=[
                    parameters(motion_keys),
                    {"ps4_state_topic": LaunchConfiguration("state_topic")},
                ],
                output="screen",
            ),
        ]
    )
