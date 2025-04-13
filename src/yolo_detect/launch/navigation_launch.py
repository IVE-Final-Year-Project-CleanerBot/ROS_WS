# navigation_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_detect',
            executable='bottle_navigation_smach',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['--ros-args', '--log-level', 'error']  # 设置日志级别为 error
        ),
        Node(
            package='yolo_detect',
            executable='bottle_detector',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['--ros-args', '--log-level', 'error']  # 设置日志级别为 error
        )
    ])