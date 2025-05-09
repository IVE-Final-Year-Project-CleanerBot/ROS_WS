import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    yolo_detect_dir = get_package_share_directory('yolo_detect')

    yolo_detect = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [yolo_detect_dir, '/launch', '/yolo_launch.py']),
    )
    yolo_detect_delay = launch.actions.TimerAction(period=5.0, actions=[yolo_detect])
    
    return launch.LaunchDescription([
        yolo_detect_delay,
    ])