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

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    my_nav2_dir = get_package_share_directory('nav2_ros2')
    behavior_tree = get_package_share_directory('behavior_tree')
    rviz2_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(my_nav2_dir, 'maps', 'my_home.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(my_nav2_dir, 'config', 'nav2_params.yaml'))
    behavior_tree_file = os.path.join(behavior_tree, 'config', 'recycle_bt.xml')
    
    # angle_min = LaunchConfiguration('angle_min', default='-1.57')  # 默认 -90°
    # angle_max = LaunchConfiguration('angle_max', default='1.57')   # 默认 90°

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, '/launch', '/bringup_launch.py']
        ),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path,
            # 'angle_min': angle_min,  # 传递 angle_min
            # 'angle_max': angle_max   # 传递 angle_max
        }.items(),
    )

    behavior_tree_launch = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'--bt_xml_file', behavior_tree_file},
            {'plugin_lib_names': [
            'check_for_bottles_bt_node',
            'control_arm_bt_node',
            'stop_navigation_bt_node',
            'approach_object_bt_node',
            'resume_navigation_bt_node'
            ]}
        ],
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
        # DeclareLaunchArgument(
        #     'angle_min',
        #     default_value='-1.57',
        #     description='Minimum angle for laser scan (in radians)'
        # ),
        # DeclareLaunchArgument(
        #     'angle_max',
        #     default_value='1.57',
        #     description='Maximum angle for laser scan (in radians)'
        # ),
        bringup_launch,
        rviz2_launch,
        behavior_tree_launch,
    ])
