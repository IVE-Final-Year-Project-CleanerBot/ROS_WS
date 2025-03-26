import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    my_nav2_dir = get_package_share_directory('nav2_ros2')
    rviz2_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(my_nav2_dir, 'maps', 'my_home.yaml'))
    nav2_param_path = LaunchConfiguration('map', default=os.path.join(my_nav2_dir, 'config', 'nav2_params.yaml'))

    # planner_yaml = os.path.join(get_package_share_directory('nav2_ros2'), 'config', 'planner_server.yaml')
    # costmap_config = os.path.join(get_package_share_directory('nav2_ros2'), 'config', 'costmap.yaml')
    
    # lifecycle_node = Node(
    #         package='nav2_lifecycle_manager',
    #         executable='lifecycle_manager',
    #         name='lifecycle_manager_pathplanner',
    #         output='screen',
    #         parameters=[{'use_sim_time': use_sim_time},
    #                     {'autostart': True},
    #                     {'node_names': ['planner_server']}])
    
    # planner_node = TimerAction(
    #     period=5.0,  # 延迟启动 (seconds)
    #     actions=[
    #         Node(
    #         package='nav2_planner',
    #         executable='planner_server',
    #         name='planner_server',
    #         output='screen',
    #         parameters=[planner_yaml, costmap_config])
    #     ]
    # )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, '/launch', '/bringup_launch.py']
        ),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path
        }.items(),
    )

    rviz2_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'map',
            default_value=map_yaml_path,
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_param_path,
            description='Full path to param file to load'),
        bringup_launch,
        rviz2_launch,
    ])
