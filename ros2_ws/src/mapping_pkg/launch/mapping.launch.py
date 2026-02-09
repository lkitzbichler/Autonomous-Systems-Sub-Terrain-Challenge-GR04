import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mapping_pkg_dir = get_package_share_directory('mapping_pkg')
    params_file = os.path.join(mapping_pkg_dir, 'config', 'octomap_params.yaml')

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('cloud_in', '/realsense/depth/points')
        ]
    )

    return LaunchDescription([
        octomap_server_node,
    ])
