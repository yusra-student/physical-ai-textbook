from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    pkg_share_dir = get_package_share_directory('code_examples') # Assuming code_examples is a ROS package
    default_model_path = os.path.join(pkg_share_dir, 'module1', 'robot_description', 'humanoid.urdf.xacro')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Path to robot URDF file'
    )

    robot_description = Command(['xacro ', LaunchConfiguration('model')])

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Gazebo Node
    gazebo_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'humanoid', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        rsp_node,
        gazebo_node
    ])
