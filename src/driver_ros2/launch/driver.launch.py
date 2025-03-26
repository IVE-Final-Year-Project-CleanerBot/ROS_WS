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
        parameters = [{'use_sim_time': use_sim_time}],
        output='screen',
        )
    
    motor_control_node = Node(
        package='driver_ros2',
        executable='motor_control_node',
        name='motor_control_node',
        parameters = [{'use_sim_time': use_sim_time}],
        output='screen',
    )
    
    battery_status_node = Node(
        package='driver_ros2',
        executable='battery_status_node',
        name='battery_status_node',
        parameters = [{'use_sim_time': use_sim_time}],
        output='screen',
    )

    twist_motor_control_node = Node(
        package='driver_ros2',
        executable='twist_motor_control_node',
        name='twist_motor_control_node',
        parameters = [{'use_sim_time': use_sim_time}],
        output='screen',
    )

    pwm_servo_control_node = Node(
        package='driver_ros2',
        executable='pwm_servo_control_node',
        name='pwm_servo_control_node',
        parameters = [{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        driver_node,
        motor_control_node,
        battery_status_node,
        twist_motor_control_node,
        pwm_servo_control_node,
    ])