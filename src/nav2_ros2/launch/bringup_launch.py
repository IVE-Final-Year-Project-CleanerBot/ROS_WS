from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('sllidar_ros2'), 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'cartographer_launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2_launch.py'))
        ),
        Node(
            package='sllidar_ros2',
            executable='your_robot_controller',
            name='robot_controller',
            output='screen'
        )
    ])