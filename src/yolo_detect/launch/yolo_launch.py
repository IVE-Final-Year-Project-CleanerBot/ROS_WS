import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    bottle_detection_node = Node(
        package='yolo_detect',
        executable='bottle_detection_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    bottle_detection_node_delay = launch.actions.TimerAction(
        period=5.0,  # Delay in seconds
        actions=[bottle_detection_node]
    )

    midas_node = Node(
        package='yolo_detect',
        executable='midas_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    bottle_nav2position_node = Node(
        package='yolo_detect',
        executable='bottle_nav2position_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    bottle_nav2position_node_delay = launch.actions.TimerAction(
        period=10.0,  # Delay in seconds
        actions=[bottle_nav2position_node]
    )

    return LaunchDescription([
        bottle_detection_node_delay,
        midas_node,
        bottle_nav2position_node_delay
    ])