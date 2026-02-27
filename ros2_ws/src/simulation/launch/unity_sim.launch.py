from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the Unity binary installed by ExternalProject into share/unity_bridge/unitysim
    pkg_share = get_package_share_directory('unity_bridge')
    unity_bin = os.path.join(pkg_share, 'unitysim', 'VNAV.x86_64')

    vnav = ExecuteProcess(
        cmd=[unity_bin],
        output='screen'
    )

    w_to_unity = Node(
        package='unity_bridge',
        executable='w_to_unity',
        name='w_to_unity',
        output='screen',
        # You can override the UDP params here if you like:
        # parameters=[{'ip_address': '127.0.0.1', 'port': '12346'}]
    )

    unity_state = Node(
        package='unity_bridge',
        executable='unity_state',
        name='unity_state',
        output='screen'
    )

    return LaunchDescription([vnav, w_to_unity, unity_state])
