from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="statemachine_pkg",
            executable="state_machine_node",
            name="state_machine_node",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])
