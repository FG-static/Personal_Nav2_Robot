import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_tunnel_guidance')
    config = os.path.join(pkg_share, 'config', 'tunnel_guidance.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_auto_goal = LaunchConfiguration('enable_auto_goal', default='false')
    auto_goal_dwell_time = LaunchConfiguration(
        'auto_goal_dwell_time', default='8.0')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_enable_auto_goal = DeclareLaunchArgument(
        'enable_auto_goal', default_value='false',
        description='Automatically send tunnel guidance local goals to Nav2')
    declare_auto_goal_dwell_time = DeclareLaunchArgument(
        'auto_goal_dwell_time', default_value='8.0',
        description='Seconds to wait after reaching each auto goal')

    node = Node(
        package='my_tunnel_guidance',
        executable='tunnel_guidance_node',
        name='tunnel_guidance',
        output='screen',
        parameters=[config, {
            'use_sim_time': use_sim_time,
            'enable_auto_goal': enable_auto_goal,
            'auto_goal_dwell_time': auto_goal_dwell_time,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_auto_goal,
        declare_auto_goal_dwell_time,
        node,
    ])
