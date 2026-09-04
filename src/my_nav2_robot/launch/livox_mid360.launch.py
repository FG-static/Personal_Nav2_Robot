import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_nav2_robot')
    default_config = os.path.join(pkg_share, 'config', 'MID360_config.json')

    use_sim_time = LaunchConfiguration('use_sim_time')
    user_config_path = LaunchConfiguration('user_config_path')
    frame_id = LaunchConfiguration('frame_id')
    publish_freq = LaunchConfiguration('publish_freq')

    # xfer_format 0 = sensor_msgs/PointCloud2, matching /livox/lidar consumers
    # in this stack (pointcloud_to_laserscan, tunnel guidance, BIEVR-LIO).
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 0,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': publish_freq,
            'output_data_type': 0,
            'frame_id': frame_id,
            'user_config_path': user_config_path,
            'cmdline_input_bd_code': 'livox0000000001',
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Hardware Mid360 uses wall clock, not /clock'),
        DeclareLaunchArgument(
            'user_config_path',
            default_value=default_config,
            description='MID360 JSON: host_* IPs must match the NIC running this node'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='livox_frame',
            description='frame_id written into PointCloud2 / Imu headers'),
        DeclareLaunchArgument(
            'publish_freq',
            default_value='10.0',
            description='Point cloud publish frequency in Hz'),
        livox_driver,
    ])
