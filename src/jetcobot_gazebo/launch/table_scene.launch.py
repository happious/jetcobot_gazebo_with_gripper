"""Table scenes. The robot scene starts the existing Gazebo controllers and MoveIt."""
from pathlib import Path
import os
import shlex
import tempfile
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as Arg
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def setup(context):
    package = Path(share('jetcobot_gazebo'))
    description = Path(share('jetcobot_description'))
    with_robot = Arg('robot').perform(context).lower() in ('true', '1')
    gui = Arg('gui').perform(context).lower() in ('true', '1')
    world = ET.parse(package / 'worlds' / ('table_robot_cube.world' if with_robot else 'table_cube.world'))
    controllers = str(package / 'config/ros2_controllers.yaml')
    robot_text = None
    if with_robot:
        robot = ET.parse(package / 'urdf/jetcobot_table.urdf')
        # Resolve installed description resources, including workspaces moved after build.
        for mesh in robot.iter('mesh'):
            filename = mesh.get('filename', '')
            if 'jetcobot_description/meshes/' in filename:
                mesh.set('filename', (description / 'meshes' / filename.split('jetcobot_description/meshes/', 1)[1]).as_uri())
            elif filename.startswith('model://table_marble/'):
                mesh.set('filename', (package / 'models/table_marble' / filename.split('model://table_marble/', 1)[1]).as_uri())
        robot.find('gazebo/plugin/parameters').text = controllers
        robot_text = ET.tostring(robot.getroot(), encoding='unicode')
        # Inline the preconverted robot into a temporary world, resolving package paths.
        model = ET.parse(package / 'models/jetcobot_table/model.sdf').getroot().find('model')
        model.set('name', 'jetcobot')
        for uri in model.iter('uri'):
            if uri.text and 'jetcobot_description/meshes/' in uri.text:
                uri.text = str(description / 'meshes' / uri.text.split('jetcobot_description/meshes/', 1)[1])
        model.find("plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']/parameters").text = controllers
        scene = world.getroot().find('world')
        for include in list(scene.findall('include')):
            if include.findtext('uri') == 'model://jetcobot_table':
                scene.remove(include)
        scene.append(model)
    temporary = tempfile.TemporaryDirectory(prefix='jetcobot_table_')
    filename = Path(temporary.name) / 'scene.sdf'
    world.write(filename, encoding='unicode', xml_declaration=True)
    # Retain and clean the generated file at launch shutdown.
    cleanup = RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: temporary.cleanup()))
    resources = os.pathsep.join([str(package / 'models'), str(description.parent), os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('ros_gz_sim')) / 'launch/gz_sim.launch.py')), launch_arguments={'gz_args': '-r ' + ('' if gui else '-s --headless-rendering ') + shlex.quote(str(filename)), 'on_exit_shutdown': 'true'}.items())
    actions = [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources), cleanup, gazebo,
               Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')]
    if not with_robot:
        return actions
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', name='wrist_camera_bridge', arguments=['/jetcobot/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image', '/jetcobot/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'], parameters=[{'use_sim_time': True}], output='screen'))
    actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': robot_text, 'use_sim_time': True}], output='screen'))
    def spawner(name):
        return Node(package='controller_manager', executable='spawner', arguments=[name, '-c', '/controller_manager', '--param-file', controllers, '--controller-manager-timeout', '90'], output='screen')
    broadcaster = spawner('joint_state_broadcaster')
    arm = spawner('arm_group_controller')
    config = MoveItConfigsBuilder('jetcobot', package_name='jetcobot_moveit_config').to_moveit_configs()
    config.robot_description = {'robot_description': robot_text}
    semantic = ET.fromstring(config.robot_description_semantic['robot_description_semantic'])
    # world is already the URDF root; do not add the old world -> dummy virtual joint.
    for joint in list(semantic.findall('virtual_joint')):
        semantic.remove(joint)
    ET.SubElement(semantic, 'disable_collisions', link1='table_link', link2='base_link', reason='Adjacent')
    config.robot_description_semantic = {'robot_description_semantic': ET.tostring(semantic, encoding='unicode')}
    move_group = Node(package='moveit_ros_move_group', executable='move_group', parameters=[config.to_dict(), {'use_sim_time': True, 'publish_robot_description': True, 'publish_robot_description_semantic': True, 'publish_planning_scene': True, 'publish_geometry_updates': True, 'publish_state_updates': True, 'publish_transforms_updates': True}], condition=IfCondition(Arg('moveit')), output='screen')
    rviz = Node(package='rviz2', executable='rviz2', arguments=['-d', str(config.package_path / 'config/moveit.rviz')], parameters=[config.robot_description, config.robot_description_semantic, config.robot_description_kinematics, config.planning_pipelines, config.joint_limits, {'use_sim_time': True}], condition=IfCondition(Arg('rviz')), output='screen')
    def after(action, following):
        def completed(event, context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='Table scene controller startup failed'))]
            return following
        return RegisterEventHandler(OnProcessExit(target_action=action, on_exit=completed))
    actions.extend([after(broadcaster, [arm]), after(arm, [move_group, rviz]), broadcaster])
    return actions


def generate_launch_description():
    defaults = {'robot': 'true', 'gui': 'true', 'moveit': 'true', 'rviz': 'true'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] + [OpaqueFunction(function=setup)])
