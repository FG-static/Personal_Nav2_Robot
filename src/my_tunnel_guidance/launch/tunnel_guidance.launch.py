import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_tunnel_guidance')
    config = os.path.join(pkg_share, 'config', 'tunnel_guidance.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_auto_goal = LaunchConfiguration('enable_auto_goal')
    auto_goal_dwell_time = LaunchConfiguration('auto_goal_dwell_time')
    allow_capture_done = LaunchConfiguration('allow_capture_done')
    wait_for_vision = LaunchConfiguration('wait_for_vision')
    wait_for_dataset = LaunchConfiguration('wait_for_dataset')
    dataset_output_dir = LaunchConfiguration('dataset_output_dir')

    node = Node(
        package='my_tunnel_guidance',
        executable='tunnel_guidance_node',
        name='tunnel_guidance',
        output='screen',
        parameters=[config, {
            'use_sim_time': use_sim_time,
            'enable_auto_goal': enable_auto_goal,
            'auto_goal_dwell_time': auto_goal_dwell_time,
            'allow_capture_done': allow_capture_done,
            'wait_for_vision': wait_for_vision,
            'wait_for_dataset': wait_for_dataset,
            'dataset_output_dir': dataset_output_dir,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'),
        DeclareLaunchArgument(
            'enable_auto_goal',
            default_value='true',
            description='Automatically send tunnel guidance local goals to Nav2'),
        DeclareLaunchArgument(
            'auto_goal_dwell_time',
            default_value='1.0',
            description='Seconds to wait after reaching each auto goal'),
        DeclareLaunchArgument(
            'allow_capture_done',
            default_value='false',
            description='Wait for MCU capture_done on /tracker/gimbal before vision capture'),
        DeclareLaunchArgument(
            'wait_for_vision',
            default_value='false',
            description='After MCU capture_done, command the vision node and wait for 0x02'),
        DeclareLaunchArgument(
            'wait_for_dataset',
            default_value='true',
            description='Wait until the local inspection dataset is saved before departing'),
        DeclareLaunchArgument(
            'dataset_output_dir',
            default_value='/tmp/tunnel_inspections',
            description='Directory for per-station clouds and merged map.pcd'),
        node,
    ])
