import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav2_dir = get_package_share_directory('nav2_ros2')
    nav2_commander_dir = get_package_share_directory('nav2_commander')

    nav2 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_dir, '/launch', '/navigation_launch.py']),
    )
    nav2_delay = launch.actions.TimerAction(period=5.0, actions=[nav2])

    nav2_commander = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_commander_dir, '/launch', '/nav2_commander_launch.py']),
    )
    nav2_commander_delay = launch.actions.TimerAction(period=5.0, actions=[nav2_commander])
    
    return launch.LaunchDescription([
        nav2_delay,
        nav2_commander_delay,
    ])