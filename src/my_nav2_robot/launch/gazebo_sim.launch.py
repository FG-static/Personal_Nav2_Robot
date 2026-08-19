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
    slam = LaunchConfiguration('slam', default='False')
    use_gazebo_odom_tf = LaunchConfiguration('use_gazebo_odom_tf', default='false')
    spawn_culvert = LaunchConfiguration('spawn_culvert', default='true')
    spawn_map1_obstacles = LaunchConfiguration('spawn_map1_obstacles', default='true')

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether to enable SLAM mode'
    )
    use_gazebo_odom_tf_arg = DeclareLaunchArgument(
        'use_gazebo_odom_tf',
        default_value='false',
        description=(
            'Publish Gazebo ground-truth odom TF. Keep false when an external '
            'odometry source such as BIEVR-LIO publishes odom -> base_footprint.'
        )
    )
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

    # 机器人初始位置：
    #   culvert -> 左侧入口处（涵洞沿 X 轴，左端 x=-12.0；车身略向内放 0.2m）
    #   map1    -> 地图中心
    robot_spawn_x = PythonExpression([
        "'-11.8' if '", scene, "' == 'culvert' else '0.0'"
    ])

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_cool_robot',
            '-allow_renaming', 'true',
            '-x', robot_spawn_x,
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
                   # /scan 不再从 Gazebo 2D 雷达桥接，改由 Mid360 点云投影生成。
                   '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                   '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                   '/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',  # 绝对真实里程计
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],  # 这个确实要单向
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Optional fallback: bridge Gazebo's ground-truth odom TF into ROS.
    # Do not enable this when BIEVR-LIO is running, because both sources would
    # publish the same dynamic odom -> base_footprint transform.
    gazebo_odom_tf_condition = PythonExpression([
        "'True' if '", slam, "' in ('true', 'True') and '",
        use_gazebo_odom_tf, "' in ('true', 'True') else 'False'"
    ])
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(gazebo_odom_tf_condition)
    )

    # Mid360 点云 -> /scan，供 AMCL / SLAM Toolbox / costmap 继续用 LaserScan。
    pointcloud_to_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'pointcloud_to_laserscan.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
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
        slam_arg,
        use_gazebo_odom_tf_arg,
        scene_arg,
        spawn_culvert_arg,
        spawn_map1_obstacles_arg,
        gazebo,
        spawn_entity,
        spawn_culvert_node,
        spawn_map1_obstacles,
        bridge,
        tf_bridge,
        pointcloud_to_scan,
        simulated_imu,
        node_robot_state_publisher,
    ])
