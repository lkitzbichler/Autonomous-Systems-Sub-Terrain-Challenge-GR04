from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare("path_planning_pkg"),
        "config",
        "path_planning_params.yaml",
    ])

    pathplanner_node = Node(
        package="path_planning_pkg",
        executable="pathplanner_node",
        name="path_planner",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        pathplanner_node,
    ])
