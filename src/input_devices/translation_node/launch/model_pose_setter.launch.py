from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        "namespace": "robot1",
        "bind_ip": "127.0.0.1",
        "bind_port": "5550",
        "depth_server_ip": "127.0.0.1",
        "depth_server_port": "5555",
        "use_interlaced_image": "false",
        "rectify_images": "false",
        "crop_images": "false",
        "use_koala_grasp": "false",
        "gripper_transport": "service",
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
        "camera_frame": "endoscope/camera_optical_frame",
        "movement_time": "0.5",
        "calibration_path": "package://storz_endoscope_calibration.yml",
    }
    types = {
        "bind_port": int,
        "depth_server_port": int,
        "movement_time": float,
        "use_interlaced_image": bool,
        "rectify_images": bool,
        "crop_images": bool,
        "use_koala_grasp": bool,
    }
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in defaults.items()
            ],
            Node(
                package="input_translation",
                namespace=LaunchConfiguration("namespace"),
                executable="model_robot_pose_setter",
                output="screen",
                parameters=[
                    {
                        name: ParameterValue(
                            LaunchConfiguration(name), value_type=types.get(name, str)
                        )
                        for name in defaults
                        if name != "namespace"
                    }
                ],
            ),
        ]
    )
