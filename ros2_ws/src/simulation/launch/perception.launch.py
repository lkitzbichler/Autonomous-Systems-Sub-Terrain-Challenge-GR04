from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    depth_info_topic = LaunchConfiguration('depth_info_topic')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    lantern_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('lantern_detector_pkg'), 'launch', 'lantern_detector.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('depth_image_topic', default_value='/realsense/depth/image'),
        DeclareLaunchArgument('depth_info_topic', default_value='/realsense/depth/camera_info'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/realsense/depth/points'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

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
        lantern_detector_launch,
    ])
