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
            executable='bottle_navigation_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='yolo_detect',
            executable='bottle_pickup_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='yolo_detect',
            executable='bottle_place_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])