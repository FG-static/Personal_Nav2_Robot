import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_tunnel_guidance')
    config = os.path.join(pkg_share, 'config', 'tunnel_guidance.yaml')

    node = Node(
        package='my_tunnel_guidance',
        executable='tunnel_guidance_node',
        name='tunnel_guidance',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([node])
