import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the launch directory
    pkg_name = 'my_robot_bringup' # You would typically create a package for your robot
    # For now, we'll assume the xacro is in the current package structure
    
    # Path to your xacro file
    xacro_file = os.path.join(
        get_package_share_directory('YOUR_ROS2_PACKAGE_NAME'), # Replace with your package name
        'models',
        'simple_robot.urdf.xacro'
    )
    # The above path needs to be adjusted if not using a ROS 2 package for the xacro.
    # For this example, we'll hardcode the path relative to the current directory.
    
    # Path to your xacro file (adjusted for direct use in code-examples/module2/models)
    # This assumes the launch file is in code-examples/module2/launch
    robot_description_path = os.path.join(
        os.getcwd(),
        'code-examples',
        'module2',
        'models',
        'simple_robot.urdf.xacro'
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output='screen',
        parameters=[{'robot_description': robot_description_path, 'use_sim_time': True}]
    )

    # Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': os.path.join(
            os.getcwd(),
            'code-examples',
            'module2',
            'worlds',
            'empty_world.xml' # We need to create an empty_world.xml later
        )}.items()
    )

    # Spawn Robot
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'simple_robot'],
                        output='screen')

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity
    ])
