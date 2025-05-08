import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:

    # Start the demo autonomy task
    demo_cmd = Node(
        package='nav2_simple_commander',
        executable='picking',
        emulate_tty=True,
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(demo_cmd)
    return ld