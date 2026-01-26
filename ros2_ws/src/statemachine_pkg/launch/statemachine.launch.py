from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("statemachine_pkg"), "config", "statemachine_params.yaml"]
    )

    return LaunchDescription([
        Node(
            package="statemachine_pkg",
            executable="state_machine_node",
            name="state_machine_node",
            output="screen",
            parameters=[params_file, {"use_sim_time": True}],
        ),
    ])
