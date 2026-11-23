"""Spawn joint_rcm_controller using explicit controller parameters."""
from tempfile import NamedTemporaryFile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULTS = {'namespace': 'robot1', 'robot_type': 'ur5e', 'base_frame': 'base_link', 'tip_frame': 'tool0', 'ee_frame_tip_point': '[0.0, 0.0, 0.3]', 'ee_frame_rcm_point': '[0.0, 0.0, 0.1]', 'target_tip_pose_topic': 'set_tip_pose', 'cartesian_velocity_limit': '0.05', 'cartesian_acceleration_limit': '0.25', 'cartesian_jerk_limit': '1.0'}


def setup(context):
    values = {key: LaunchConfiguration(key).perform(context) for key in DEFAULTS}
    namespace = values.pop('namespace').strip('/')
    values['frame_prefix'] = f'{namespace}/' if namespace else ''
    for key in ('ee_frame_tip_point', 'ee_frame_rcm_point'):
        values[key] = yaml.safe_load(values[key])
    for key in ('cartesian_velocity_limit', 'cartesian_acceleration_limit', 'cartesian_jerk_limit'):
        if key in values:
            values[key] = float(values[key])
    name = f'/{namespace}/joint_rcm_controller' if namespace else '/joint_rcm_controller'
    with NamedTemporaryFile(mode='w', suffix='.yaml', prefix='testbed-joint_rcm_controller-', delete=False) as stream:
        yaml.safe_dump({name: {'ros__parameters': values}}, stream)
        parameter_file = stream.name
    return [Node(package='controller_manager', executable='spawner', namespace=namespace,
        arguments=['joint_rcm_controller', '--param-file', parameter_file,
                   '--controller-manager', f'/{namespace}/controller_manager' if namespace else '/controller_manager',
                   '--controller-manager-timeout', '30'], output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value)
                              for key, value in DEFAULTS.items()] + [OpaqueFunction(function=setup)])
