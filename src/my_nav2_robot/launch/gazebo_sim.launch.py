import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'my_nav2_robot'
    pkg_share = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    map1_obstacles_file = os.path.join(
        pkg_share, 'models', 'map1_obstacles', 'model.sdf')
    culvert_file = os.path.join(pkg_share, 'models', 'culvert.sdf')
    robot_description = Command(['xacro ', xacro_file])

    # 选择 Gazebo 中要生成的静态场景：
    #   scene:=culvert -> 4m 宽、2m 高、10m 长的涵洞两侧墙（不封底）
    #   scene:=map1    -> 原来的 map1 6 个障碍物
    scene = LaunchConfiguration('scene', default='culvert')
    spawn_culvert = LaunchConfiguration('spawn_culvert', default='true')
    spawn_map1_obstacles = LaunchConfiguration('spawn_map1_obstacles', default='true')

    scene_arg = DeclareLaunchArgument(
        'scene',
        default_value='culvert',
        description='Static scene to spawn: culvert or map1'
    )
    spawn_culvert_arg = DeclareLaunchArgument(
        'spawn_culvert',
        default_value='true',
        description='Spawn the culvert walls when scene is culvert'
    )
    spawn_map1_obstacles_arg = DeclareLaunchArgument(
        'spawn_map1_obstacles',
        default_value='true',
        description='Spawn map1 obstacles when scene is map1'
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
            # 地图 bounds 以 (0, 0) 为中心，所以机器人也固定生成在地图中心。
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
        output='screen'
    )

    spawn_culvert_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'sensors', '-file', culvert_file],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", scene, "' == 'culvert' and '",
            spawn_culvert, "' == 'true'"
        ]))
    )

    spawn_map1_obstacles = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'sensors', '-file', map1_obstacles_file],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", scene, "' == 'map1' and '",
            spawn_map1_obstacles, "' == 'true'"
        ]))
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
        scene_arg,
        spawn_culvert_arg,
        spawn_map1_obstacles_arg,
        gazebo,
        spawn_entity,
        spawn_culvert_node,
        spawn_map1_obstacles,
        bridge,
        simulated_imu,
        node_robot_state_publisher,
    ])
