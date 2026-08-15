import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'my_nav2_robot'
    pkg_share = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    map1_obstacles_file = os.path.join(
        pkg_share, 'models', 'map1_obstacles', 'model.sdf')
    robot_description = Command(['xacro ', xacro_file])

    spawn_map1_obstacles_arg = DeclareLaunchArgument(
        'spawn_map1_obstacles',
        default_value='true',
        description='Spawn collision geometry corresponding to maps/map1.pgm'
    )

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
            }.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_cool_robot',
            '-allow_renaming', 'true',
        ],
        output='screen'
    )

    spawn_map1_obstacles = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'sensors', '-file', map1_obstacles_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('spawn_map1_obstacles'))
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',  # 最好是这样
                   # /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry，轮式里程计
                   '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                   '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                   '/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',  # 绝对真实里程计
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],  # 这个确实要单向
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    simulated_imu = Node(
        package='my_nav2_robot',
        executable='simulated_imu_publisher',
        name='simulated_imu_publisher',
        parameters=[{
            'use_sim_time': True,
            'odometry_topic': '/ground_truth',
            'output_topic': '/livox/imu',
            'frame_id': 'imu_link',
            'gravity_magnitude': 9.8,
            'maximum_time_step': 0.1,
        }],
        output='screen'
    )

    return LaunchDescription([
        spawn_map1_obstacles_arg,
        gazebo,
        spawn_entity,
        spawn_map1_obstacles,
        bridge,
        simulated_imu,
        node_robot_state_publisher,
    ])
