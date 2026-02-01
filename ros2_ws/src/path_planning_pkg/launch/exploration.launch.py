from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare("path_planning_pkg"),
        "config",
        "exploration_params.yaml",
    ])

    exploration_node = Node(
        package="path_planning_pkg",
        executable="exploration_node",
        name="exploration_node",
        output="screen",
        parameters=[params_file],
    )

    sampler_node = Node(
        package="mav_trajectory_generation",
        executable="trajectory_sampler_node",
        name="trajectory_sampler",
        output="screen",
        remappings=[
            ("path_segments_4D", "trajectory"),
        ],
    )

    return LaunchDescription([
        exploration_node,
        sampler_node,
    ])
