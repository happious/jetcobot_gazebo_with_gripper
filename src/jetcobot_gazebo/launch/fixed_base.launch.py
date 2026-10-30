from pathlib import Path
import xml.etree.ElementTree as ET
import shlex
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as Arg
from launch_ros.actions import Node, SetParameter


def setup(context):
    package = Path(share('jetcobot_gazebo'))
    description = Path(share('jetcobot_description'))
    robot = ET.parse(package / 'urdf/jetcobot_gz_fixed_base.urdf').getroot()
    for mesh in robot.iter('mesh'):
        uri = mesh.get('filename', '')
        prefix = 'package://jetcobot_description/'
        if uri.startswith(prefix):
            mesh.set('filename', (description / uri[len(prefix):]).as_uri())
    robot.find('gazebo/plugin/parameters').text = str(package / 'config/ros2_controllers.yaml')
    controllers = str(package / 'config/ros2_controllers.yaml')
    world = Path(Arg('world').perform(context)).expanduser().resolve()
    world_element = ET.parse(world).getroot().find('world')
    if world_element is None or not world_element.get('name'):
        raise ValueError(f'World file must contain a named <world>: {world}')
    world_name = world_element.get('name')
    gui = Arg('gui').perform(context).lower() in ('true', '1')
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('ros_gz_sim')) / 'launch/gz_sim.launch.py')), launch_arguments={'gz_args': '-r ' + ('' if gui else '-s ') + shlex.quote(str(world)), 'on_exit_shutdown': 'true'}.items())
    publisher = Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': ET.tostring(robot, encoding='unicode'), 'use_sim_time': True}], output='screen')
    clock = Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')
    spawn = Node(package='ros_gz_sim', executable='create', output='screen', arguments=['-world', world_name, '-topic', '/robot_description', '-name', 'jetcobot', '-x', Arg('x'), '-y', Arg('y'), '-z', Arg('z'), '-Y', Arg('yaw')])
    def spawner(name):
        return Node(package='controller_manager', executable='spawner', output='screen', arguments=[name, '-c', '/controller_manager', '--param-file', controllers, '--controller-manager-timeout', '60'])
    broadcaster = spawner('joint_state_broadcaster')
    arm = spawner('arm_group_controller')
    launch_dir = Path(share('jetcobot_moveit_config')) / 'launch'
    def include(name, **kwargs):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir / name)), **kwargs)
    moveit = GroupAction(condition=IfCondition(Arg('moveit')), actions=[SetParameter(name='use_sim_time', value=True), include('move_group.launch.py'), include('moveit_rviz.launch.py', condition=IfCondition(Arg('rviz')))])
    def after(action, next_action):
        def completed(event, context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='JetCobot startup stage failed'))]
            return [next_action]
        return RegisterEventHandler(OnProcessExit(target_action=action, on_exit=completed))
    return [after(spawn, broadcaster), after(broadcaster, arm), after(arm, moveit), publisher, gazebo, clock, spawn]


def generate_launch_description():
    defaults = {'gui': 'true', 'moveit': 'false', 'rviz': 'true', 'x': '0', 'y': '0', 'z': '0', 'yaw': '0',
                'world': str(Path(share('jetcobot_gazebo')) / 'worlds/empty.world')}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] + [OpaqueFunction(function=setup)])
