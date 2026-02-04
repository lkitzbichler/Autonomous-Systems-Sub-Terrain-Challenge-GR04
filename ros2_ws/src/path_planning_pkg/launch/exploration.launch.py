from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('path_planning_pkg')
    
    # Declare launch arguments
    exploration_height = DeclareLaunchArgument(
        'exploration_height',
        default_value='2.0',
        description='Target height for frontier exploration')
    
    height_margin = DeclareLaunchArgument(
        'height_margin',
        default_value='3.0',
        description='Margin above/below drone height for slices')
    
    map_topic = DeclareLaunchArgument(
        'map_topic',
        default_value='/map_3d',
        description='Topic for 3D occupancy grid')
    
    frontier_rank = DeclareLaunchArgument(
        'frontier_rank',
        default_value='0',
        description='Rank of frontier to select')

    # Frontier exploration node
    frontier_explorer_node = Node(
        package='frontier_exploration',
        executable='classical_frontier_detector',
        name='frontier_explorer',
        parameters=[
            {'region_size_thresh': 12},
            {'robot_width': 0.5},
            {'occupancy_map_msg': LaunchConfiguration('map_topic')}
        ],
        remappings=[
            ('map', LaunchConfiguration('map_topic'))
        ],
        output='screen'
    )

    # Exploration node
    exploration_node = Node(
        package='path_planning_pkg',
        executable='exploration_node',
        name='exploration_node',
        parameters=[
            os.path.join(pkg_share, 'config', 'exploration_params.yaml'),
            {'exploration_height': LaunchConfiguration('exploration_height')},
            {'height_margin': LaunchConfiguration('height_margin')},
            {'map_topic': LaunchConfiguration('map_topic')},
            {'frontier_rank': LaunchConfiguration('frontier_rank')}
        ],
        output='screen'
    )

    return LaunchDescription([
        exploration_height,
        height_margin,
        map_topic,
        frontier_rank,
        frontier_explorer_node,
        exploration_node
    ])
