from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Path to your YAML config file
    pkg_share = get_package_share_directory('controller_pkg')
    default_config = os.path.join(pkg_share, 'config', 'controller_params.yaml')

    # Optionally allow user to override config path
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to controller configuration YAML file'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock simulation time'
    )

    controller_node = Node(
        package='controller_pkg',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file'), {'use_sim_time': use_sim_time}],
        # emulate "clear_params='true'" by forcing parameter override behavior
        emulate_tty=True
    )

    return LaunchDescription([
        config_arg,
        use_sim_time_arg,
        controller_node
    ])
