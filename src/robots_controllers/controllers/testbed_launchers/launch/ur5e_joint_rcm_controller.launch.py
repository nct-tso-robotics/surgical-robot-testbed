"""Namespaced UR bringup with shared calibrated description and RCM plugins."""
import math
from copy import deepcopy
from pathlib import Path
from tempfile import NamedTemporaryFile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def deep_merge(base, override):
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value


def load_controller_config(path):
    """Evaluate the upstream template with unchanged hardware joint names."""
    context = LaunchContext()
    context.launch_configurations['tf_prefix'] = ''
    parameters = ParameterFile(path, allow_substs=True)
    try:
        with parameters.evaluate(context).open() as stream:
            return yaml.safe_load(stream)
    finally:
        parameters.cleanup()


def setup(context):
    def value(key):
        return LaunchConfiguration(key).perform(context)

    calibration_file = Path(value('kinematics_params_file')).expanduser().resolve()
    if not calibration_file.is_file():
        raise FileNotFoundError(f'Robot calibration file does not exist: {calibration_file}')
    namespace = value('namespace').strip('/')
    frame_prefix = f'{namespace}/' if namespace else ''
    package = Path(get_package_share_directory('testbed_launchers'))
    driver = Path(get_package_share_directory('ur_robot_driver'))
    config = load_controller_config(driver / 'config/ur_controllers.yaml')
    offsets = {name: [float(coordinate) for coordinate in yaml.safe_load(value(name))]
               for name in ('ee_frame_tip_point', 'ee_frame_rcm_point')}
    if any(len(point) != 3 or not all(math.isfinite(coordinate) for coordinate in point) for point in offsets.values()):
        raise ValueError('Tip and pivot offsets must each contain three finite coordinates')
    common = {'robot_type': value('robot_type'), 'base_frame': value('base_frame'), 'frame_prefix': frame_prefix,
              'tip_frame': value('tip_frame'), **offsets}
    deep_merge(config, {
      'controller_manager': {'ros__parameters': {'update_rate': 500,
        'rcm_broadcaster': {'type': 'testbed_broadcasters/RcmBroadcaster'},
        'joint_rcm_controller': {'type': 'testbed_joint_rcm_controller/JointRcmController'}}},
      'rcm_broadcaster': {'ros__parameters': {**deepcopy(common), 'read_effort': True, 'read_wrench': True}},
      'joint_rcm_controller': {'ros__parameters': {**deepcopy(common), 'target_tip_pose_topic': value('target_tip_pose_topic'),
        **{key: float(value(key)) for key in ('cartesian_velocity_limit', 'cartesian_acceleration_limit', 'cartesian_jerk_limit')}}}})
    with NamedTemporaryFile(mode='w', suffix='.yaml', prefix='testbed-ur-', delete=False) as stream:
        yaml.safe_dump({f'/{namespace}': config} if namespace else config, stream)
        controller_file = stream.name
    arguments = {key: value(key) for key in ('namespace', 'robot_ip', 'ur_type', 'reverse_port',
                 'trajectory_port', 'script_command_port', 'script_sender_port')}
    arguments['kinematics_params_file'] = str(calibration_file)
    arguments.update({'controllers_file': controller_file, 'tf_prefix': '', 'frame_prefix': frame_prefix,
                      'launch_rviz': 'false'})
    control = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package / 'launch/ur_control.launch.py')),
                                      launch_arguments=arguments.items())
    tip = Node(package='tf2_ros', executable='static_transform_publisher', namespace=namespace,
               name='tool_tip_transform', arguments=[
                   '--x', str(offsets['ee_frame_tip_point'][0]), '--y', str(offsets['ee_frame_tip_point'][1]),
                   '--z', str(offsets['ee_frame_tip_point'][2]), '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', frame_prefix + value('tip_frame'), '--child-frame-id', frame_prefix + 'tool_tip'])
    spawn = Node(package='controller_manager', executable='spawner', namespace=namespace,
                 arguments=['joint_state_broadcaster', 'io_and_status_controller', 'speed_scaling_state_broadcaster',
                            'rcm_broadcaster', 'joint_rcm_controller', '--activate-as-group',
                            '--controller-manager', f'/{namespace}/controller_manager' if namespace else '/controller_manager',
                            '--controller-manager-timeout', '60'], output='screen')
    return [control, tip, spawn]


def generate_launch_description():
    defaults = {'namespace': 'robot1', 'ur_type': 'ur5e', 'robot_type': 'ur5e',
                'base_frame': 'base_link', 'tip_frame': 'tool0',
                'ee_frame_tip_point': '[0.0, 0.0, 0.3]', 'ee_frame_rcm_point': '[0.0, 0.0, 0.1]',
                'target_tip_pose_topic': 'set_tip_pose', 'reverse_port': '50001', 'trajectory_port': '50006',
                'script_command_port': '50004', 'script_sender_port': '50002',
                'cartesian_velocity_limit': '0.05', 'cartesian_acceleration_limit': '0.25', 'cartesian_jerk_limit': '1.0'}
    return LaunchDescription([DeclareLaunchArgument('robot_ip', description='Explicit physical robot address'),
        DeclareLaunchArgument('kinematics_params_file', description='Calibration YAML extracted from this robot')]
        + [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
        + [OpaqueFunction(function=setup)])
