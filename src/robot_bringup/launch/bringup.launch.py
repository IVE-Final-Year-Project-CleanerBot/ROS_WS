import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bot_bringup_dir = get_package_share_directory(
        'robot_bringup')
    lidar_ros2_dir = get_package_share_directory(
        'sllidar_ros2')

    urdf2tf = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [bot_bringup_dir, '/launch', '/urdf2tf.launch.py']),
    )

    odom2tf = launch_ros.actions.Node(
        package='robot_bringup',
        executable='odom2tf',
        output='screen'
    )

    # map2odomtf = launch_ros.actions.Node(
    #     package='robot_bringup',
    #     executable='map2odomtf',
    #     output='screen'
    # )

    lidar = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [lidar_ros2_dir, '/launch', '/sllidar_c1_launch.py']),
    )

    # 使用 TimerAction 启动后 5 秒执行 lidar 节点
    lidar_delay = launch.actions.TimerAction(period=5.0, actions=[lidar])
    
    return launch.LaunchDescription([
        urdf2tf,
        odom2tf,
        # map2odomtf,
        lidar_delay
    ])