"""Articulated physical gripper: no attachment plugin and no collision exclusions."""
from pathlib import Path
import shlex
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as Arg
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def setup(context):
    package=Path(share('jetcobot_gazebo'))
    moveit_package=Path(share('jetcobot_moveit_config'))
    description=Path(share('jetcobot_description'))
    robot=ET.parse(package/'urdf/jetcobot_articulated.urdf').getroot()
    if Arg('initial_home').perform(context).lower() in ('true', '1'):
        semantic = ET.parse(moveit_package/'config/jetcobot.srdf').getroot()
        for joint in semantic.findall("group_state[@name='home'][@group='arm_group']/joint"):
            initial = robot.find(f"ros2_control/joint[@name='{joint.get('name')}']/state_interface[@name='position']/param[@name='initial_value']")
            if initial is None:
                raise RuntimeError(f"Missing initial position for {joint.get('name')}")
            initial.text = joint.get('value')
    for mesh in robot.iter('mesh'):
        uri=mesh.get('filename','')
        if uri.startswith('package://'):
            pkg,relative=uri[len('package://'):].split('/',1)
            path=Path(share(pkg))/relative
            if not path.is_file():raise FileNotFoundError(path)
            mesh.set('filename',path.as_uri())
    native=Arg('native_mimic').perform(context).lower() in ('true','1')
    engine=Arg('physics_engine').perform(context)
    if native and 'bullet-featherstone' not in engine:
        raise ValueError('native_mimic requires the installed Bullet Featherstone engine')
    for entry in robot.findall('ros2_control/joint'):
        source=robot.find(f"joint[@name='{entry.get('name')}']")
        if source is not None and source.find('mimic') is not None:
            entry.set('mimic','false' if native else 'true')
    if native:
        # The Isaac damping (0.02 Nm s/rad) is numerically excessive for
        # ~1e-6 kg m2 links under Bullet's explicit damping integration.
        for joint in robot.findall('joint'):
            if joint.get('name')=='gripper_controller' or joint.find('mimic') is not None:
                dynamics=joint.find('dynamics')
                dynamics.set('damping','0.0001')
    controllers=str(package/'config'/('articulated_controllers_native.yaml' if native else 'articulated_controllers.yaml'))
    robot.find('gazebo/plugin/parameters').text=controllers
    text=ET.tostring(robot,encoding='unicode')
    world=Path(Arg('world').perform(context)).expanduser().resolve()
    world_name=ET.parse(world).getroot().find('world').get('name')
    gui=Arg('gui').perform(context).lower() in ('true','1')
    gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('ros_gz_sim'))/'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args':'-r '+('' if gui else '-s ')+'--physics-engine '+shlex.quote(engine)+' '+shlex.quote(str(world)),'on_exit_shutdown':'true'}.items())
    publisher=Node(package='robot_state_publisher',executable='robot_state_publisher',
        parameters=[{'robot_description':text,'use_sim_time':True}],output='screen')
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],output='screen')
    # Native mode uses physical gear constraints. Servo fallback is explicit,
    # never enabled at the same time as native coupling.
    with tempfile.NamedTemporaryFile(suffix='.urdf',mode='w') as file:
        file.write(text);file.flush()
        converted=subprocess.run(['gz','sdf','-p',file.name],text=True,capture_output=True,check=True).stdout
    sdf=ET.fromstring(converted)
    if not native:
        for axis in sdf.findall('.//joint/axis'):
            for mimic in list(axis.findall('mimic')):axis.remove(mimic)
    if native:
        # Bullet Featherstone's default triangle-mesh path adds a 10mm margin,
        # which swallows the clearance of this small hand. Convex decomposition
        # preserves concavities using multiple hulls with the engine's 1mm margin.
        for link in sdf.findall('model/link'):
            if not link.get('name','').startswith('gripper'):continue
            for mesh in link.findall('collision/geometry/mesh'):
                mesh.set('optimization','convex_decomposition')
                decomposition=ET.SubElement(mesh,'convex_decomposition')
                ET.SubElement(decomposition,'max_convex_hulls').text='16'
                ET.SubElement(decomposition,'voxel_resolution').text='200000'
    # Contact sensors observe real physics contacts; no forces or joints are added.
    for side in ('left','right'):
        link=sdf.find(f"model/link[@name='gripper_{side}1']")
        if link is None: raise RuntimeError('Missing fingertip link for contact sensor')
        sensor=ET.SubElement(link,'sensor',name=f'{side}_finger_contact',type='contact')
        ET.SubElement(sensor,'always_on').text='true'
        ET.SubElement(sensor,'update_rate').text='100'
        ET.SubElement(sensor,'topic').text=f'/jetcobot/contacts/{side}'
        contact=ET.SubElement(sensor,'contact')
        ET.SubElement(contact,'topic').text=f'/jetcobot/contacts/{side}'
        ET.SubElement(contact,'collision').text=link.find('collision').get('name')
    spawn=Node(package='ros_gz_sim',executable='create',
        arguments=['-world',world_name,'-string',ET.tostring(sdf,encoding='unicode'),'-name','jetcobot'],output='screen')
    def controller(name):
        return Node(package='controller_manager',executable='spawner',output='screen',
            arguments=[name,'-c','/controller_manager','--param-file',controllers,'--controller-manager-timeout','90','--switch-timeout','120'])
    broadcaster=controller('joint_state_broadcaster');arm=controller('arm_group_controller');gripper=controller('gripper_group_controller')
    config=(MoveItConfigsBuilder('jetcobot',package_name='jetcobot_moveit_config')
        .robot_description(file_path=str(package/'urdf/jetcobot_articulated.urdf'))
        .robot_description_semantic(file_path='config/jetcobot.srdf')
        .to_moveit_configs())
    config.robot_description={'robot_description':text}
    config.robot_description_semantic={'robot_description_semantic':(moveit_package/'config/jetcobot.srdf').read_text()}
    config.joint_limits['robot_description_planning']['joint_limits']['gripper_controller']={
        'has_velocity_limits':True,'max_velocity':2.0,'has_acceleration_limits':True,'max_acceleration':4.0}
    config.trajectory_execution['moveit_simple_controller_manager']['controller_names'].append('gripper_group_controller')
    config.trajectory_execution['moveit_simple_controller_manager']['gripper_group_controller']={
        'type':'FollowJointTrajectory','joints':['gripper_controller'],
        'action_ns':'follow_joint_trajectory','default':True}
    move_group=Node(package='moveit_ros_move_group',executable='move_group',
        parameters=[config.to_dict(),{'use_sim_time':True,'publish_robot_description':True,
            'publish_robot_description_semantic':True,'publish_planning_scene':True}],
        condition=IfCondition(Arg('moveit')),output='screen')
    rviz=Node(package='rviz2',executable='rviz2',arguments=['-d',str(config.package_path/'config/moveit.rviz')],
        parameters=[config.robot_description,config.robot_description_semantic,
            config.robot_description_kinematics,config.planning_pipelines,config.joint_limits,{'use_sim_time':True}],
        condition=IfCondition(Arg('rviz')),output='screen')
    def after(action,following):
        def done(event,context):
            if event.returncode:return [EmitEvent(event=Shutdown(reason='Articulated model startup failed'))]
            return following
        return RegisterEventHandler(OnProcessExit(target_action=action,on_exit=done))
    return [after(spawn,[broadcaster]),after(broadcaster,[arm]),after(arm,[gripper]),
            after(gripper,[move_group,rviz]),publisher,bridge,gazebo,spawn]


def generate_launch_description():
    package=Path(share('jetcobot_gazebo'))
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in {
        'physics_engine':'gz-physics-bullet-featherstone-plugin','native_mimic':'true',
        'gui':'true','moveit':'true','rviz':'true','initial_home':'true','world':str(package/'worlds/robot_only.world')}.items()]+[OpaqueFunction(function=setup)])
