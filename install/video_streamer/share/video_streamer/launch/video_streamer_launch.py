import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    video_stream_node = Node(
        package='video_streamer',
        executable='video_stream',
        name='video_stream_node',
        parameters = [{'use_sim_time': use_sim_time}],
        output='screen',
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        video_stream_node,
    ])