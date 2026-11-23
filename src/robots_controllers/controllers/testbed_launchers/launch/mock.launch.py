"""Hardware-free UR5e loop using ros2_control GenericSystem velocity integration."""
import math
from pathlib import Path
from tempfile import NamedTemporaryFile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    frame_prefix = f'{namespace}/' if namespace else ''
    offsets = {name: [float(value) for value in yaml.safe_load(LaunchConfiguration(name).perform(context))]
               for name in ('ee_frame_tip_point', 'ee_frame_rcm_point')}
    if any(len(point) != 3 or not all(math.isfinite(value) for value in point) for point in offsets.values()):
        raise ValueError('Tip and pivot offsets must each contain three finite coordinates')
    with NamedTemporaryFile(mode='w', suffix='.yaml', prefix='testbed-mock-', delete=False) as stream:
        yaml.safe_dump({f'/{namespace}/{name}' if namespace else f'/{name}':
                       {'ros__parameters': {'frame_prefix': frame_prefix,
                                            **{key: point.copy() for key, point in offsets.items()}}}
                       for name in ('rcm_broadcaster', 'joint_rcm_controller')}, stream)
        controller_parameters = stream.name
    description = (Path(get_package_share_directory('testbed_launchers')) / 'urdf/mock_ur5e.urdf').read_text()
    manager = Node(package='controller_manager', executable='ros2_control_node', namespace=namespace,
                   parameters=[{'update_rate': 100, 'lock_memory': False,
                                'joint_state_broadcaster.type': 'joint_state_broadcaster/JointStateBroadcaster',
                                'rcm_broadcaster.type': 'testbed_broadcasters/RcmBroadcaster',
                                'joint_rcm_controller.type': 'testbed_joint_rcm_controller/JointRcmController'}],
                   output='screen')
    state = Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=namespace,
                 parameters=[{'robot_description': description, 'frame_prefix': frame_prefix}], output='screen')
    tip = Node(package='tf2_ros', executable='static_transform_publisher', namespace=namespace,
               name='tool_tip_transform', arguments=[
                   '--x', str(offsets['ee_frame_tip_point'][0]), '--y', str(offsets['ee_frame_tip_point'][1]),
                   '--z', str(offsets['ee_frame_tip_point'][2]), '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', frame_prefix + 'tool0', '--child-frame-id', frame_prefix + 'tool_tip'])
    spawner = Node(package='controller_manager', executable='spawner', namespace=namespace,
                   arguments=['joint_state_broadcaster', 'rcm_broadcaster', 'joint_rcm_controller',
                              '--param-file', controller_parameters,
                              '--controller-manager', f'/{namespace}/controller_manager' if namespace else '/controller_manager',
                              '--controller-manager-timeout', '30', '--activate-as-group'], output='screen')
    return [manager, state, tip, spawner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot1'),
        DeclareLaunchArgument('ee_frame_tip_point', default_value='[0.0, 0.0, 0.3]'),
        DeclareLaunchArgument('ee_frame_rcm_point', default_value='[0.0, 0.0, 0.1]'),
        OpaqueFunction(function=setup)])
