import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Paths to URDF and world files
    package_name = 'task1'
    urdf_file_path = os.path.join(
        get_package_share_directory(package_name), 'urdf', 'bot_desc.urdf'
    )
    world_file_path = os.path.join(
        get_package_share_directory(package_name), 'worlds', 'empty.world'
    )

    # Read URDF file content
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description = urdf_file.read()
    
    # Properly format YAML for SpawnEntity service call
    spawn_entity_args = f"name: my_robot\nxml: '{robot_description}'"

    return LaunchDescription([
        # Start Gazebo server with specified world file
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Spawn robot in Gazebo using SpawnEntity service
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spawn_entity_args],
            output='screen'
        ),
    ])




