from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    driver_node = Node(
        package='driver_ros2',
        executable='driver_node',
        name='driver_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    bottle_pickup_node = Node(
        package='driver_ros2',
        executable='bottle_pickup_node',
        name='bottle_pickup_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    bottle_place_node = Node(
        package='driver_ros2',
        executable='bottle_place_node',
        name='bottle_place_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        driver_node,
        bottle_pickup_node,
        bottle_place_node,
    ])