#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Launch configs (ROS1 <arg> equivalents)
    load_params = LaunchConfiguration("load_params")
    corrupt_state_estimate = LaunchConfiguration("corrupt_state_estimate")

    right_image_topic = LaunchConfiguration("right_image_topic")
    right_info_topic = LaunchConfiguration("right_info_topic")
    left_image_topic = LaunchConfiguration("left_image_topic")
    left_info_topic = LaunchConfiguration("left_info_topic")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    depth_info_topic = LaunchConfiguration("depth_info_topic")

    # Declare args
    declared_args = [
        DeclareLaunchArgument("load_params", default_value="true"),
        DeclareLaunchArgument("corrupt_state_estimate", default_value="true"),
        DeclareLaunchArgument("right_image_topic", default_value="/realsense/rgb/right_image_raw"),
        DeclareLaunchArgument("right_info_topic", default_value="/realsense/rgb/right_image_info"),
        DeclareLaunchArgument("left_image_topic", default_value="/realsense/rgb/left_image_raw"),
        DeclareLaunchArgument("left_info_topic", default_value="/realsense/rgb/left_image_info"),
        DeclareLaunchArgument("depth_image_topic", default_value="/realsense/depth/image"),
        DeclareLaunchArgument("depth_info_topic", default_value="/realsense/depth/camera_info"),
    ]

    # <include file="$(find simulation)/launch/unity_ros.launch"> ...
    # Assumes the included file is also a ROS2 Python launch file named unity_ros.launch.py
    unity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("simulation"), "launch", "unity_ros.launch.py"])
        ),
        launch_arguments={
            "load_params": load_params,
            "right_image_topic": right_image_topic,
            "right_info_topic": right_info_topic,
            "left_image_topic": left_image_topic,
            "left_info_topic": left_info_topic,
            "depth_image_topic": depth_image_topic,
            "depth_info_topic": depth_info_topic,
        }.items(),
    )

    # Nodes
    simulation_node = Node(
        package="simulation",
        executable="Simulation.x86_64",
        name="Simulation",
        output="screen",
    )

    state_estimate_corruptor = Node(
        package="simulation",
        executable="state_estimate_corruptor_node",
        name="state_estimate_corruptor",
        output="screen",
        parameters=[
            # drift_rw_factor
            {"drift_rw_factor": 0.03},
            # pos_white_sig
            {"pos_white_sig": 0.005},
            # jump_seconds
            {"jump_seconds": 20.0},
        ],
        condition=IfCondition(corrupt_state_estimate),
    )

    # Same node but with "unless": set params to the "disabled" values
    state_estimate_corruptor_disabled = Node(
        package="simulation",
        executable="state_estimate_corruptor_node",
        name="state_estimate_corruptor",
        output="screen",
        parameters=[
            {"drift_rw_factor": 0.0},
            {"pos_white_sig": 0.0},
            {"jump_seconds": -1.0},
        ],
        condition=UnlessCondition(corrupt_state_estimate),
    )

    w_to_unity = Node(
        package="simulation",
        executable="w_to_unity",
        name="w_to_unity",
        output="screen",
    )

    controller_node = Node(
        package="controller_pkg",
        executable="controller_node",
        name="controller_node",
        output="screen",
    )

    # Static TF publishers (ROS2 CLI style args; verify for your ROS2 distro)
    static_tf_nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sim_true_body",
            arguments=["0", "0", "0", "0", "0", "0", "/Quadrotor/TrueState", "/true_body"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sim_rgb_camera",
            arguments=["0", "-0.05", "0", "0", "0", "0", "/camera", "/Quadrotor/RGBCameraLeft"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sim_depth_camera",
            arguments=["0", "0", "0", "0", "0", "0", "/depth_camera", "/Quadrotor/DepthCamera"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sim_right_camera",
            arguments=["0", "0.05", "0", "0", "0", "0", "/camera", "/Quadrotor/RGBCameraRight"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_to_body",
            arguments=["0", "0", "0", "0", "0", "0", "/true_body", "/camera"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="depth_camera_to_body",
            arguments=["0", "0", "0", "0", "0", "0", "/true_body", "/depth_camera"],
            output="screen",
        ),
    ]

    return LaunchDescription(
        declared_args
        + [
            unity_launch,
            simulation_node,
            state_estimate_corruptor,
            state_estimate_corruptor_disabled,
            w_to_unity,
            controller_node,
            *static_tf_nodes,
        ]
    )
