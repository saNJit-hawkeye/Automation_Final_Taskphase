import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = 'task1'

    # Correcting file paths
    world_file = os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.world')
    urdf_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'bot_desc.urdf')


    # Launch Gazebo 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments=[('world', world_file)]
    )

    # Spawnning robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_file, '-entity', 'task1'],
        output='screen'
    )

    return LaunchDescription([gazebo, spawn_robot])




