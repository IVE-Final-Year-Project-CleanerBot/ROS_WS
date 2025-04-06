from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_stream',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            parameters=[]  # 如果有参数，可以在这里添加
        ),
    ])