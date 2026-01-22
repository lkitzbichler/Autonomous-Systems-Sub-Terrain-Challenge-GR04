from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    depth_info_topic = LaunchConfiguration('depth_info_topic')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')

    return LaunchDescription([
        DeclareLaunchArgument('depth_image_topic', default_value='/realsense/depth/image_rect_raw'),
        DeclareLaunchArgument('depth_info_topic', default_value='/realsense/depth/camera_info'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/realsense/depth/points'),

        ComposableNodeContainer(
            name='perception_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[
                        ('image_rect', depth_image_topic),
                        ('camera_info', depth_info_topic),
                        ('points', pointcloud_topic),
                    ],
                ),
            ],
            output='screen',
        ),
    ])
