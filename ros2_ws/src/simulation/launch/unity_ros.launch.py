#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configs
    load_params = LaunchConfiguration("load_params")

    pose_topic = LaunchConfiguration("pose_topic")
    right_image_topic = LaunchConfiguration("right_image_topic")
    right_info_topic = LaunchConfiguration("right_info_topic")
    left_image_topic = LaunchConfiguration("left_image_topic")
    left_info_topic = LaunchConfiguration("left_info_topic")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    depth_info_topic = LaunchConfiguration("depth_info_topic")
    imu_topic = LaunchConfiguration("imu_topic")

    declared_args = [
        DeclareLaunchArgument("load_params", default_value="true"),
        DeclareLaunchArgument("pose_topic", default_value="/true_pose"),
        DeclareLaunchArgument("right_image_topic", default_value="/realsense/rgb/image_rect_raw_right"),
        DeclareLaunchArgument("right_info_topic", default_value="/realsense/rgb/camera_info_right"),
        DeclareLaunchArgument("left_image_topic", default_value="/realsense/rgb/image_rect_raw_left"),
        DeclareLaunchArgument("left_info_topic", default_value="/realsense/rgb/camera_info_left"),
        DeclareLaunchArgument("depth_image_topic", default_value="/realsense/depth/image_rect_raw"),
        DeclareLaunchArgument("depth_info_topic", default_value="/realsense/depth/camera_info"),
        DeclareLaunchArgument("imu_topic", default_value="/interpolate_imu/imu"),
    ]

    unity_ros_node = Node(
        package="simulation",
        executable="unity_ros",
        name="unity_ros",
        output="screen",
        ros_arguments=[
            "--ros-args",
            "-r", ["Quadrotor/Sensors/IMU/pose:=", pose_topic],
            "-r", "Quadrotor/Sensors/IMU/twist:=/true_twist",
            "-r", ["Quadrotor/Sensors/DepthCamera/image_raw:=", depth_image_topic],
            "-r", ["Quadrotor/Sensors/DepthCamera/camera_info:=", depth_info_topic],
            "-r", ["Quadrotor/Sensors/RGBCameraLeft/image_raw:=", left_image_topic],
            "-r", ["Quadrotor/Sensors/RGBCameraLeft/camera_info:=", left_info_topic],
            "-r", ["Quadrotor/Sensors/RGBCameraRight/image_raw:=", right_image_topic],
            "-r", ["Quadrotor/Sensors/RGBCameraRight/camera_info:=", right_info_topic],
            "-r", ["Quadrotor/IMU:=", imu_topic],
        ],
        parameters=[],
    )

    return LaunchDescription(declared_args + [unity_ros_node])
