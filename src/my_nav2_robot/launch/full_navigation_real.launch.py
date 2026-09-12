import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Nav2 + SLAM bringup for the real robot (no Gazebo).

    Sensor data sources are started here (hardware_bringup.launch.py):
      - livox_ros_driver2 on the real Mid360 -> /livox/lidar + /livox/imu
      - pointcloud_to_laserscan              -> /scan

    Odometry comes from BIEVR-LIO. The Ceres in bievr_ws conflicts with the
    Nav2 environment, so BIEVR runs in a separate terminal:

        source /home/nav/bievr_ws/quick_source.sh
        ros2 launch bievr_lio_ros2 process_topics.launch.py \
            sensor_config:=nav2_real params:=params rviz:=false

    BIEVR publishes /bievr_lio/odom and the TF odom -> base_footprint that
    this launch's Nav2 stack consumes (odom_topic is rewritten accordingly).

    tunnel:=true additionally starts the my_tunnel_guidance auto inspection
    node (allow_capture_done waits for the MCU capture_done handshake).

    Not started here: gzserver/gzclient, Mid360 Gazebo plugin, simulated IMU,
    and ground-truth odom.
    """
    pkg_project = get_package_share_directory('my_nav2_robot')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_serial = get_package_share_directory('rm_serial_driver')
    pkg_tunnel = get_package_share_directory('my_tunnel_guidance')

    slam_mode = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    run_serial = LaunchConfiguration('serial')
    run_tunnel = LaunchConfiguration('tunnel')
    allow_capture_done = LaunchConfiguration('allow_capture_done')
    chassis = LaunchConfiguration('chassis')
    start_livox = LaunchConfiguration('start_livox')
    start_scan = LaunchConfiguration('start_scan')
    map_override = LaunchConfiguration('map')
    scene = LaunchConfiguration('scene')

    slam_for_bringup = PythonExpression([
        "'True' if '", slam_mode, "' in ('true', 'True') else 'False'"
    ])

    nav2_params_slam_diff = os.path.join(
        pkg_project, 'config', 'nav2_params_slam_diff.yaml')
    nav2_params_slam_mecanum = os.path.join(
        pkg_project, 'config', 'nav2_params_slam.yaml')
    nav2_params = PythonExpression([
        "'", nav2_params_slam_diff, "' if '", chassis,
        "' == 'diff' else '", nav2_params_slam_mecanum, "'"
    ])
    bt_xml_diff = os.path.join(pkg_project, 'behavior_trees', 'test_nav_diff.xml')
    bt_xml_mecanum = os.path.join(pkg_project, 'behavior_trees', 'test_nav.xml')
    default_nav_to_pose_bt_xml = PythonExpression([
        "'", bt_xml_diff, "' if '", chassis,
        "' == 'diff' else '", bt_xml_mecanum, "'"
    ])
    rviz_config = os.path.join(pkg_project, 'config', 'nav2_config.rviz')
    tunnel_config = os.path.join(pkg_tunnel, 'config', 'tunnel_guidance.yaml')

    # Always BIEVR-LIO on the real robot. No ground-truth or wheel-odom TF.
    params_rewritten = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
            'odom_topic': '/bievr_lio/odom',
            'tf_broadcast': 'False',
        },
        convert_types=True,
    )

    map_yaml_file = PythonExpression([
        "'", os.path.join(pkg_project, 'maps'), '/', scene, ".yaml' if '",
        map_override, "' == '' else '",
        map_override, "'"
    ])
    dynamic_map_path = PythonExpression([
        "'' if '", slam_mode, "' in ('true', 'True') else '",
        map_yaml_file, "'"
    ])

    # Mid360 lidar + IMU source, /scan projection, robot TF.
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project, 'launch', 'hardware_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'chassis': chassis,
            'start_livox': start_livox,
            'start_scan': start_scan,
        }.items(),
    )

    # slam:=False only: identity map->odom. slam_toolbox publishes it in SLAM.
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map',
                   '--child-frame-id', 'odom'],
        condition=UnlessCondition(slam_mode),
    )

    # slam_toolbox only publishes /map while at least one subscriber exists.
    map_keepalive = Node(
        package='my_nav2_robot',
        executable='map_keepalive.py',
        name='map_keepalive',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(slam_mode),
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': slam_for_bringup,
            'map': dynamic_map_path,
            'params_file': params_rewritten,
            'use_composition': 'False',
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    serial_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_serial, 'launch', 'serial_driver.launch.py')
        ),
        condition=IfCondition(run_serial),
        launch_arguments={
            'device_name': LaunchConfiguration('device_name'),
        }.items(),
    )

    tunnel_guidance = Node(
        package='my_tunnel_guidance',
        executable='tunnel_guidance_node',
        name='tunnel_guidance',
        output='screen',
        condition=IfCondition(run_tunnel),
        parameters=[
            tunnel_config,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'allow_capture_done': ParameterValue(
                    allow_capture_done, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Run slam_toolbox (True) or AMCL + map_server (False).'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Must stay false on the real robot (no /clock).'),
        DeclareLaunchArgument(
            'chassis',
            default_value='diff',
            description='Chassis kinematics: diff (default) or mecanum.'),
        DeclareLaunchArgument(
            'start_livox',
            default_value='true',
            description='Start livox_ros_driver2 (real Mid360 lidar + IMU).'),
        DeclareLaunchArgument(
            'start_scan',
            default_value='true',
            description='Project /livox/lidar onto /scan.'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Run RViz.'),
        DeclareLaunchArgument(
            'scene',
            default_value='culvert',
            description='Map name when slam:=False: culvert or map1.'),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Explicit map YAML when slam:=False; empty uses maps/<scene>.yaml.'),
        DeclareLaunchArgument(
            'serial',
            default_value='true',
            description='Start rm_serial_driver so /cmd_vel goes to the MCU.'),
        DeclareLaunchArgument(
            'device_name',
            default_value='/dev/ttyACM0',
            description='USB CDC device for rm_serial_driver.'),
        DeclareLaunchArgument(
            'tunnel',
            default_value='false',
            description='Start tunnel_guidance auto inspection.'),
        DeclareLaunchArgument(
            'allow_capture_done',
            default_value='true',
            description=(
                'When tunnel:=true, wait for the MCU capture_done handshake '
                'before the next goal. Keep true on the real robot.')),
        hardware,
        static_map_to_odom,
        map_keepalive,
        nav2_bringup,
        serial_driver,
        tunnel_guidance,
        rviz_node,
    ])
