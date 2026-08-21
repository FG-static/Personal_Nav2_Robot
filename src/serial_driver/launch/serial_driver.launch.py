import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rm_serial_driver'), 'config', 'serial_driver.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device_name',
            default_value='/dev/ttyACM0',
            description='USB CDC serial device connected to the MCU'),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Nav2 / teleop Twist topic forwarded to the MCU'),
        DeclareLaunchArgument(
            'cmd_send_hz',
            default_value='20.0',
            description='Chassis command serial TX rate'),
        Node(
            package='rm_serial_driver',
            executable='rm_serial_driver_node',
            name='serial_driver',
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=[
                config,
                {
                    'device_name': LaunchConfiguration('device_name'),
                    'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                    'cmd_send_hz': ParameterValue(
                        LaunchConfiguration('cmd_send_hz'), value_type=float),
                },
            ],
        ),
    ])
