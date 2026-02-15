import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = os.path.join(
        get_package_share_directory('lantern_detector_pkg'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock simulation time",
        ),
        Node(
            package='lantern_detector_pkg',
            executable='lantern_detector',
            name='lantern_detector',
            output='screen',
            parameters=[config, {"use_sim_time": use_sim_time}]
        )
    ])
