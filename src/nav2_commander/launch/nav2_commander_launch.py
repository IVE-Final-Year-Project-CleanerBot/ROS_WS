from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_commander',
            executable='init_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='nav2_commander',
            executable='target_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])