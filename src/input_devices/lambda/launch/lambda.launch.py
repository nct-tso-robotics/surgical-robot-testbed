from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        "namespace": "robot1",
        "hand": "any",
        "device_index": "-1",
        "device_serial": "-1",
        "fast_init": "false",
        "lambda_state_topic": "lambda/state",
        "lambda_force_topic": "lambda/force",
        "force_timeout": "0.1",
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
        "pedal_board_topic": "pedal_board/state",
        "clutch_pedal": "top_grey_pedal",
        "require_clutch": "true",
        "input_position_scaling": "100.0",
        "input_rotation_scaling": "50.0",
        "movement_time": "0.5",
        "control_mode": "tip_pose",
        "safety_limits_xyz": "[0.05, 0.05, 0.05]",
        "safety_roll_limit": "0.05",
        "camera_coordinates_control": "false",
        "camera_frame": "endoscope/camera_optical_frame",
        "use_koala_grasp": "false",
        "gripper_transport": "service",
        "koala_command_topic": "/koala_controller/command",
        "gripper_command_topic": "/koala_grasper/open",
    }
    types = {
        "device_index": int,
        "device_serial": int,
        "fast_init": bool,
        "force_timeout": float,
        "input_position_scaling": float,
        "input_rotation_scaling": float,
        "movement_time": float,
        "require_clutch": bool,
        "safety_limits_xyz": None,
        "safety_roll_limit": float,
        "camera_coordinates_control": bool,
        "use_koala_grasp": bool,
    }

    def parameters(names):
        return {
            name: ParameterValue(
                LaunchConfiguration(name), value_type=types.get(name, str)
            )
            for name in names
        }

    device_keys = [
        "hand",
        "device_index",
        "device_serial",
        "fast_init",
        "lambda_state_topic",
        "lambda_force_topic",
        "force_timeout",
    ]
    motion_keys = [
        name for name in defaults if name != "namespace" and name not in device_keys
    ] + ["lambda_state_topic"]
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(
                    name,
                    default_value=value,
                    choices=["any", "left", "right"] if name == "hand" else None,
                )
                for name, value in defaults.items()
            ],
            Node(
                package="lambda",
                executable="lambda",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[parameters(device_keys)],
            ),
            Node(
                package="input_translation",
                executable="lambda_robot_pose_setter",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[parameters(motion_keys)],
            ),
        ]
    )
