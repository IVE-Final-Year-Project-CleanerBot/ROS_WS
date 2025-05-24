from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_detect',
            executable='bottle_detection_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='yolo_detect',
            executable='midas_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='yolo_detect',
            executable='nav2position_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])