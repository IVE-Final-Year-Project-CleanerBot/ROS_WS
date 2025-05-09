import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav2_dir = get_package_share_directory('nav2_ros2')
    yolo_detect_dir = get_package_share_directory('yolo_detect')

    nav2 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_dir, '/launch', '/navigation_launch.py']),
    )
    nav2_delay = launch.actions.TimerAction(period=5.0, actions=[nav2])

    yolo_detect = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [yolo_detect_dir, '/launch', '/yolo_launch.py']),
    )
    yolo_detect_delay = launch.actions.TimerAction(period=5.0, actions=[yolo_detect])
    
    return launch.LaunchDescription([
        nav2_delay,
        yolo_detect_delay,
    ])