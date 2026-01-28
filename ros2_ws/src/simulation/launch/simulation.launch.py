#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import math

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
    controller_config_file = LaunchConfiguration("controller_config_file")

    # Declare args
    controller_config_source = PathJoinSubstitution(
        [FindPackageShare("controller_pkg"), "config", "controller_params.yaml"]
    )

    declared_args = [
        DeclareLaunchArgument("load_params", default_value="true"),
        DeclareLaunchArgument("corrupt_state_estimate", default_value="true"),
        DeclareLaunchArgument("right_image_topic", default_value="/realsense/rgb/right_image_raw"),
        DeclareLaunchArgument("right_info_topic", default_value="/realsense/rgb/right_image_info"),
        DeclareLaunchArgument("left_image_topic", default_value="/realsense/rgb/left_image_raw"),
        DeclareLaunchArgument("left_info_topic", default_value="/realsense/rgb/left_image_info"),
        DeclareLaunchArgument("depth_image_topic", default_value="/realsense/depth/image"),
        DeclareLaunchArgument("depth_info_topic", default_value="/realsense/depth/camera_info"),
        DeclareLaunchArgument(
            "controller_config_file",
            default_value=controller_config_source,
        ),
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

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("simulation"), "launch", "perception.launch.py"])
        ),
        launch_arguments={
            "depth_image_topic": depth_image_topic,
            "depth_info_topic": depth_info_topic,
        }.items(),
    )

    waypoint_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("basic_waypoint_pkg"), "launch", "waypoint_mission.launch.py"])
        )
    )
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("mapping_pkg"), "launch", "mapping.launch.py"])
        )
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

    unity_state = Node(
        package="simulation",
        executable="unity_state",
        name="unity_state",
        output="screen",
    )

    controller_node = Node(
        package="controller_pkg",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[controller_config_file],
    )

    # Static TF publishers
    static_tf_nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sim_rgb_camera",
            arguments=[
                "--x", "0", "--y", "-0.05", "--z", "0",
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", "camera",
                "--child-frame-id", "Quadrotor/Sensors/RGBCameraLeft"
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sim_depth_camera",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", "depth_camera",
                "--child-frame-id", "Quadrotor/Sensors/DepthCamera"
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sim_semantic_camera",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", "depth_camera",
                "--child-frame-id", "Quadrotor/Sensors/SemanticCamera"
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sim_right_camera",
            arguments=[
                "--x", "0", "--y", "0.05", "--z", "0",
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", "camera",
                "--child-frame-id", "Quadrotor/Sensors/RGBCameraRight"
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_to_body",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--yaw", "0", "--pitch", "0", "--roll", "1.5708", # Apply 90 deg roll for optical frame
                "--frame-id", "true_body",
                "--child-frame-id", "camera"
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="depth_camera_to_body",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--yaw", "-1.5708", "--pitch", "0", "--roll", "-1.5708", # Apply 90 deg roll for optical frame
                "--frame-id", "true_body",
                "--child-frame-id", "depth_camera"
            ],
            output="screen",
        ),
    ]

    return LaunchDescription(
        declared_args
        + [
            unity_launch,
            perception_launch,
            mapping_launch,
            simulation_node,
            state_estimate_corruptor,
            state_estimate_corruptor_disabled,
            w_to_unity,
            unity_state,
            controller_node,
            *static_tf_nodes,
            waypoint_launch,
        ]
    )
