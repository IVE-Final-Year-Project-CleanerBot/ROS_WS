from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 获取 URDF 文件路径
    urdf_package_path = get_package_share_directory('robot_description')
    default_urdf_path = os.path.join(urdf_package_path, 'urdf', 'robot.urdf')
    default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')

    # Get model urdf content
    substitutions_command_result = Command(['cat ', LaunchConfiguration('model')])
    robot_description_value = ParameterValue(substitutions_command_result, value_type=str)

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_value}]
        )
    
    joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        )
    
    rviz2_launch = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
    )
    

    # 启动 robot_state_publisher 节点
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=str(default_urdf_path),
            description='Loading model file path'),
        
        robot_state_publisher,
        joint_state_publisher,
        rviz2_launch
    ])