# navigation_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_detect',
            executable='bottle_navigation_smach',
            output='screen'
        ),
        Node(
            package='yolo_detect',
            executable='bottle_detector',
            output='screen'
        )
    ])