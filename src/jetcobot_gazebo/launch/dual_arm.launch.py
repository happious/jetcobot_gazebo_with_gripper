"""Launch two parallel JetCobots in Gazebo with one shared MoveIt scene."""
from copy import deepcopy
from pathlib import Path
import shlex
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as Arg
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


ARM_JOINTS = [f"{i}_Joint" for i in range(1, 7)]
CONTROLLER_NAMES = ["arm_left_controller", "gripper_left_controller",
                    "arm_right_controller", "gripper_right_controller"]


def setup(context):
    package = Path(share("jetcobot_gazebo"))
    description = Path(share("jetcobot_description"))
    spacing_y = 0.40
    robot = ET.parse(package / "urdf/dual/jetcobot_dual.urdf").getroot()
    control_source = ET.parse(package / "urdf/jetcobot_articulated.urdf").getroot()
    source_control = control_source.find("ros2_control")
    combined_control = ET.Element("ros2_control", {"name": "GazeboSimSystem", "type": "system"})
    combined_control.append(deepcopy(source_control.find("hardware")))

    # Replace the model anchors so changing the launch source always preserves
    # the agreed 40 cm center-to-center Y spacing and equal base yaw.
    for side, y in (("left", spacing_y / 2), ("right", -spacing_y / 2)):
        root_joint = robot.find(f"joint[@name='{side}_world_to_dummy']")
        if root_joint is None:
            raise RuntimeError(f"Missing {side} base anchor in dual URDF")
        root_joint.find("origin").set("xyz", f"0 {y:.3f} 0.1")

        for source_joint in source_control.findall("joint"):
            joint = deepcopy(source_joint)
            joint_name = f"{side}_{source_joint.get('name')}"
            joint.set("name", joint_name)
            model_joint = control_source.find(f"joint[@name='{source_joint.get('name')}']")
            mimic_joint = model_joint is not None and model_joint.find("mimic") is not None
            if mimic_joint:
                joint.set("mimic", "true")
            combined_control.append(joint)

        for source_gz in control_source.findall("gazebo[@reference]"):
            element = deepcopy(source_gz)
            element.set("reference", f"{side}_{source_gz.get('reference')}")
            robot.append(element)

    robot.append(combined_control)
    controllers_yaml = package / "config/dual_controllers.yaml"
    gazebo_plugin = ET.SubElement(robot, "gazebo")
    plugin = ET.SubElement(gazebo_plugin, "plugin", {
        "filename": "libgz_ros2_control-system.so",
        "name": "gz_ros2_control::GazeboSimROS2ControlPlugin"})
    ET.SubElement(plugin, "parameters").text = str(controllers_yaml)

    for mesh in robot.iter("mesh"):
        uri = mesh.get("filename", "")
        if uri.startswith("package://"):
            package_name, relative = uri[len("package://"):].split("/", 1)
            mesh_path = Path(share(package_name)) / relative
            if not mesh_path.is_file():
                raise FileNotFoundError(mesh_path)
            mesh.set("filename", mesh_path.as_uri())

    urdf_text = ET.tostring(robot, encoding="unicode")
    world_path = Path(Arg("world").perform(context)).expanduser().resolve()
    world_name = ET.parse(world_path).getroot().find("world").get("name")
    gui = Arg("gui").perform(context).lower() in ("true", "1")
    engine = Arg("physics_engine").perform(context)
    if "dartsim" not in engine:
        raise ValueError("The dual-arm multi-tree setup requires the DART physics engine")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(share("ros_gz_sim")) / "launch/gz_sim.launch.py")),
        launch_arguments={
            "gz_args": "-r " + ("" if gui else "-s ") + "--physics-engine " +
                       shlex.quote(engine) + " " + shlex.quote(str(world_path)),
            "on_exit_shutdown": "true",
        }.items())
    clock_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen")
    robot_state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher",
                                 parameters=[{"robot_description": urdf_text, "use_sim_time": True}],
                                 output="screen")

    with tempfile.NamedTemporaryFile(suffix=".urdf", mode="w") as source_file:
        source_file.write(urdf_text)
        source_file.flush()
        sdf_text = subprocess.run(["gz", "sdf", "-p", source_file.name],
                                  text=True, capture_output=True, check=True).stdout
    sdf = ET.fromstring(sdf_text)
    # DART uses ros2_control mimic state interfaces for the parallel fingers.
    # Remove the native SDF mimic extension to avoid commanding both mechanisms.
    for axis in sdf.findall(".//joint/axis"):
        for mimic in list(axis.findall("mimic")):
            axis.remove(mimic)
    spawn = Node(package="ros_gz_sim", executable="create",
                 arguments=["-world", world_name, "-string", ET.tostring(sdf, encoding="unicode"),
                            "-name", "jetcobot_dual"], output="screen")

    def controller(name):
        return Node(package="controller_manager", executable="spawner", output="screen",
                    arguments=[name, "-c", "/controller_manager", "--param-file", str(controllers_yaml),
                               "--controller-manager-timeout", "90", "--switch-timeout", "120"])

    broadcaster = controller("joint_state_broadcaster")
    arm_left = controller("arm_left_controller")
    gripper_left = controller("gripper_left_controller")
    arm_right = controller("arm_right_controller")
    gripper_right = controller("gripper_right_controller")

    moveit = MoveItConfigsBuilder("jetcobot", package_name="jetcobot_moveit_config").to_moveit_configs()
    moveit.robot_description = {"robot_description": urdf_text}
    semantic_path = package / "config/dual/jetcobot_dual.srdf"
    moveit.robot_description_semantic = {"robot_description_semantic": semantic_path.read_text()}
    moveit.robot_description_kinematics = {
        "robot_description_kinematics": yaml.safe_load((package / "config/dual/kinematics.yaml").read_text())}
    moveit.joint_limits = {
        "robot_description_planning": yaml.safe_load((package / "config/dual/joint_limits.yaml").read_text())}
    ompl = moveit.planning_pipelines["ompl"]
    for group_name in ("arm_left", "arm_right", "both_arms", "gripper_left", "gripper_right"):
        ompl[group_name] = {"planner_configs": ["RRTConnect"]}
    moveit.planning_pipelines["ompl"] = ompl
    moveit.trajectory_execution = {
        "moveit_manage_controllers": False,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": CONTROLLER_NAMES,
        },
    }
    for side in ("left", "right"):
        arm_name = f"arm_{side}_controller"
        grip_name = f"gripper_{side}_controller"
        moveit.trajectory_execution["moveit_simple_controller_manager"][arm_name] = {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default": True,
            "joints": [f"{side}_{joint}" for joint in ARM_JOINTS],
        }
        moveit.trajectory_execution["moveit_simple_controller_manager"][grip_name] = {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default": True,
            "joints": [f"{side}_gripper_controller"],
        }
    moveit_node = Node(package="moveit_ros_move_group", executable="move_group",
                       parameters=[moveit.to_dict(), {"use_sim_time": True}],
                       condition=IfCondition(Arg("moveit")), output="screen")
    rviz = Node(package="rviz2", executable="rviz2",
                arguments=["-d", str(package / "config/dual/jetcobot_dual.rviz")],
                parameters=[moveit.to_dict(), {"use_sim_time": True}],
                condition=IfCondition(Arg("rviz")), output="screen")

    def after(action, following):
        def done(event, _context):
            if event.returncode:
                return [EmitEvent(event=Shutdown(reason=f"Dual-arm startup failed: {action}") )]
            return following
        return RegisterEventHandler(OnProcessExit(target_action=action, on_exit=done))

    return [
        after(spawn, [broadcaster]),
        after(broadcaster, [arm_left]),
        after(arm_left, [gripper_left]),
        after(gripper_left, [arm_right]),
        after(arm_right, [gripper_right]),
        after(gripper_right, [moveit_node, rviz]),
        robot_state_publisher,
        clock_bridge,
        gazebo,
        spawn,
    ]


def generate_launch_description():
    package = Path(share("jetcobot_gazebo"))
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("moveit", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("physics_engine", default_value="gz-physics-dartsim-plugin"),
        DeclareLaunchArgument("world", default_value=str(package / "worlds/dual_arm.world")),
        OpaqueFunction(function=setup),
    ])
