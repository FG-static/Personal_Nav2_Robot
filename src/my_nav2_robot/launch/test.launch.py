import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('my_nav2_robot')
    default_model_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # 声明参数
    model_arg = DeclareLaunchArgument(name='model', default_value=default_model_path,
                                      description='Absolute path to robot urdf file')

    # 调用 xacro
    robot_description = Command(['xacro ', LaunchConfiguration('model')])

    # Robot State Publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # robot_driver 节点
    robot_driver_node = Node(
        package='my_nav2_robot',
        executable='robot_driver',
        name='robot_driver',
        output='screen'
    )

    # Joint State Publisher GUI 节点 
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_driver_node,
        rviz_node
    ])