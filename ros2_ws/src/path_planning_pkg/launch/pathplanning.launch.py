from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = PathJoinSubstitution([
        FindPackageShare("path_planning_pkg"),
        "config",
        "path_planning_params.yaml",
    ])

    planner_node = Node(
        package="path_planning_pkg",
        executable="pathplanner_node",
        name="path_planner",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock simulation time",
        ),
        planner_node,
    ])
