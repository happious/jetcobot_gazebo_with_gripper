"""Fixed-base articulated robot only, home pose, no automatic motion."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    root=Path(get_package_share_directory('jetcobot_gazebo'))
    return LaunchDescription([DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('world',default_value=str(root/'worlds/robot_only.world')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'launch/articulated.launch.py')),
            launch_arguments={'gui':LaunchConfiguration('gui'),'moveit':'true','initial_home':'true',
                'rviz':LaunchConfiguration('rviz'),
                'world':LaunchConfiguration('world')}.items())])
