import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    laser_filter_dir = get_package_share_directory('laser_filters')
    cartographer_dir = get_package_share_directory('cartographer_ros2')
    nav2_dir = get_package_share_directory('nav2_ros2')
    yolo_detect_dir = get_package_share_directory('yolo_detect')
    # nav2_commander_dir = get_package_share_directory('nav2_commander')

    laser_filter = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [laser_filter_dir, '/examples', '/angular_filter_example.launch.py']),
    )
    laser_filter_delay = launch.actions.TimerAction(period=5.0, actions=[laser_filter])

    cartographer = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [cartographer_dir, '/launch', '/cartographer_launch.py']),
    )
    cartographer_delay = launch.actions.TimerAction(period=5.0, actions=[cartographer])

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

    # nav2_commander = launch.actions.IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [nav2_commander_dir, '/launch', '/nav2_commander_launch.py']),
    # )
    # nav2_commander_delay = launch.actions.TimerAction(period=5.0, actions=[nav2_commander])
    
    return launch.LaunchDescription([
        laser_filter_delay,
        cartographer_delay,
        nav2_delay,
        yolo_detect_delay,
        # nav2_commander_delay,
    ])