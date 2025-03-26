import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction

def generate_launch_description():

    planner_yaml = os.path.join(get_package_share_directory('nav2_ros2'), 'config', 'planner_server.yaml')
    costmap_config = os.path.join(get_package_share_directory('nav2_ros2'), 'config', 'costmap.yaml')
    # controller_yaml = os.path.join(get_package_share_directory('nav2_ros2'), 'config', 'controller.yaml')
    # bt_navigator_yaml = os.path.join(get_package_share_directory('nav2_ros2'), 'config', 'bt_navigator.yaml')
    # recovery_yaml = os.path.join(get_package_share_directory('nav2_ros2'), 'config', 'recovery.yaml')

    lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['planner_server']}])
    
    planner_node = TimerAction(
        period=5.0,  # 延迟启动 (seconds)
        actions=[
            Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml, costmap_config])
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        lifecycle_node,
        planner_node

        # Node(
        #     package='nav2_controller',
        #     executable='controller_server',
        #     name='controller_server',
        #     output='screen',
        #     parameters=[controller_yaml]),
            
        # Node(
        #     package='nav2_recoveries',
        #     executable='recoveries_server',
        #     name='recoveries_server',
        #     parameters=[recovery_yaml],
        #     output='screen'),

        # Node(
        #     package='nav2_bt_navigator',
        #     executable='bt_navigator',
        #     name='bt_navigator',
        #     output='screen',
        #     parameters=[bt_navigator_yaml])

    ])
