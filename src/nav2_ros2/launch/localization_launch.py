import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    map_file = os.path.join(get_package_share_directory('nav2_ros2'), 'maps', 'my_home.yaml')
    amcl_config = os.path.join(get_package_share_directory('nav2_ros2'), 'config', 'amcl_config.yaml')

    lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        )
    
    # AMCL（自适应蒙特卡洛定位）
    amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config]
        )
    
    # 地图服务器
    map_server_node = TimerAction(
        period=5.0,  # 延迟启动 (seconds)
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}, 
                            {'yaml_filename': map_file}]
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        lifecycle_node,
        amcl_node,
        map_server_node
    ])