import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lantern_detector_pkg'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='lantern_detector_pkg',
            executable='lantern_detector',
            name='lantern_detector',
            output='screen',
            parameters=[config]
        )
    ])
