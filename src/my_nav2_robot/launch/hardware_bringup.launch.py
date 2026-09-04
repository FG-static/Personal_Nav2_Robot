import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_nav2_robot')
    mecanum_xacro = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    diff_xacro = os.path.join(pkg_share, 'urdf', 'robot_diff.urdf.xacro')
    default_livox_config = os.path.join(pkg_share, 'config', 'MID360_config.json')

    use_sim_time = LaunchConfiguration('use_sim_time')
    chassis = LaunchConfiguration('chassis')
    start_livox = LaunchConfiguration('start_livox')
    start_scan = LaunchConfiguration('start_scan')
    user_config_path = LaunchConfiguration('user_config_path')

    xacro_file = PythonExpression([
        "'", diff_xacro, "' if '", chassis, "' == 'diff' else '", mecanum_xacro, "'"
    ])
    robot_description = Command(['xacro ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'livox_mid360.launch.py')
        ),
        condition=IfCondition(start_livox),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'user_config_path': user_config_path,
        }.items(),
    )

    pointcloud_to_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'pointcloud_to_laserscan.launch.py')
        ),
        condition=IfCondition(start_scan),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Must stay false on the real robot'),
        DeclareLaunchArgument(
            'chassis',
            default_value='mecanum',
            description='Chassis kinematics: mecanum or diff'),
        DeclareLaunchArgument(
            'start_livox',
            default_value='true',
            description='Start livox_ros_driver2 for the hardware Mid360'),
        DeclareLaunchArgument(
            'start_scan',
            default_value='true',
            description='Project /livox/lidar onto /scan'),
        DeclareLaunchArgument(
            'user_config_path',
            default_value=default_livox_config,
            description='MID360 JSON used by livox_ros_driver2'),
        robot_state_publisher,
        joint_state_publisher,
        livox_launch,
        pointcloud_to_scan,
    ])
