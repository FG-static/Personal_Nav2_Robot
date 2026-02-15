import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    pkg_name = 'my_nav2_robot'
    pkg_share = get_package_share_directory(pkg_name)
    nav2_params = LaunchConfiguration('nav2_params')

    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    #params_file = os.path.join(pkg_share, 'config', nav2_params)
    robot_description = Command(['xacro ', xacro_file])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': '-r sensors.sdf',
            'params_file': nav2_params,
            }.items(),
    )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                    '-name', 'my_cool_robot',
                                    '-allow_renaming', 'true'],
                        output='screen')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist', # 最好是这样
                   '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                   '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                   '/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry', # 这个可能也要单向
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], # 这个确实要单向
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        bridge,
        node_robot_state_publisher,
    ])