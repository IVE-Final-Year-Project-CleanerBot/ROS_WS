import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bot_bringup_dir = get_package_share_directory(
        'robot_bringup')
    lidar_ros2_dir = get_package_share_directory(
        'sllidar_ros2')
    driver_ros2_dir = get_package_share_directory('driver_ros2')

    # 启动 URDF 转换
    urdf2tf = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [bot_bringup_dir, '/launch', '/urdf2tf.launch.py']),
    )

    # 启动里程计到 TF 的转换
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

    # 启动激光雷达
    lidar = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [lidar_ros2_dir, '/launch', '/sllidar_c1_launch.py']),
    )

    # 使用 TimerAction 启动后 5 秒执行 lidar 节点
    lidar_delay = launch.actions.TimerAction(period=5.0, actions=[lidar])

    # 启动摄像头发布节点
    camera_publisher = launch_ros.actions.Node(
        package='image_tools',
        executable='cam2image',
        name='cam2image',
        output='screen',
        parameters=[
            {'width': 640},
            {'height': 480},
            {'frequency': 10.0},
            {'reliability': 'best_effort'}
        ],
        remappings=[
            ('/image', '/camera/image_raw')
        ]
    )

    # 启动Driver节点
    driver_nodes = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [driver_ros2_dir, '/launch', '/driver.launch.py']),
    )
    
    return launch.LaunchDescription([
        urdf2tf,
        odom2tf,
        # map2odomtf,
        lidar_delay,
        camera_publisher,
        driver_nodes,
    ])