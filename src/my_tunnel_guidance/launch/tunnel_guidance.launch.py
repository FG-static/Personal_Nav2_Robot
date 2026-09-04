import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_tunnel_guidance')
    config = os.path.join(pkg_share, 'config', 'tunnel_guidance.yaml')

    # 参数唯一来源是 config/tunnel_guidance.yaml（enable_auto_goal、
    # auto_goal_dwell_time、allow_capture_done、wait_for_dataset 等）。
    # 不再提供 launch 参数覆盖：此前 launch 默认值（如 enable_auto_goal=false）
    # 会静默压过 yaml，导致"yaml 里开了自动巡检却不生效"的配置陷阱。
    node = Node(
        package='my_tunnel_guidance',
        executable='tunnel_guidance_node',
        name='tunnel_guidance',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        node,
    ])
