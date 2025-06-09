import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    init_commander_node = Node(
        package='nav2_commander',
        executable='init_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    init_commander_node_delay = launch.actions.TimerAction(
        period=5.0,
        actions=[init_commander_node]
    )

    target_commander_node = Node(
        package='nav2_commander',
        executable='target_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    bottle_target_node = Node(
        package='nav2_commander',
        executable='bottle_target_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        # target_commander_node,
        init_commander_node_delay,
        bottle_target_node,
    ])