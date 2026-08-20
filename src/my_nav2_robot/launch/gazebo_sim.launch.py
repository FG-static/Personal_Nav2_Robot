import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _join_search_path(*chunks):
    parts = []
    for chunk in chunks:
        if not chunk:
            continue
        for item in str(chunk).split(os.pathsep):
            if item and item not in parts:
                parts.append(item)
    return os.pathsep.join(parts)


def _gazebo_classic_model_dirs():
    dirs = []
    for candidate in (
            '/usr/share/gazebo-11/models',
            '/usr/share/gazebo/models'):
        if os.path.isdir(candidate):
            dirs.append(candidate)
    return dirs


def generate_launch_description():
    pkg_name = 'my_nav2_robot'
    pkg_share = get_package_share_directory(pkg_name)
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    mecanum_xacro = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    diff_xacro = os.path.join(pkg_share, 'urdf', 'robot_diff.urdf.xacro')
    map1_obstacles_file = os.path.join(
        pkg_share, 'models', 'map1_obstacles', 'model.sdf')
    culvert_file = os.path.join(pkg_share, 'models', 'culvert.sdf')
    empty_world = os.path.join(gazebo_ros_share, 'worlds', 'empty.world')

    # Package models + system Classic models (sun, ground_plane). Keep any
    # existing GAZEBO_MODEL_PATH. Empty DATABASE_URI so gzserver does not
    # block on models.gazebosim.org before /spawn_entity is advertised.
    model_path = os.path.join(pkg_share, 'models')
    plugin_path = os.path.normpath(os.path.join(pkg_share, '..', '..', 'lib'))
    os.environ['GAZEBO_MODEL_PATH'] = _join_search_path(
        model_path,
        *_gazebo_classic_model_dirs(),
        os.environ.get('GAZEBO_MODEL_PATH', ''))
    os.environ['GAZEBO_PLUGIN_PATH'] = _join_search_path(
        plugin_path,
        os.environ.get('GAZEBO_PLUGIN_PATH', ''))
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''

    scene = LaunchConfiguration('scene', default='culvert')
    slam = LaunchConfiguration('slam', default='False')
    use_gazebo_odom_tf = LaunchConfiguration('use_gazebo_odom_tf', default='false')
    spawn_culvert = LaunchConfiguration('spawn_culvert', default='true')
    spawn_map1_obstacles = LaunchConfiguration('spawn_map1_obstacles', default='true')
    chassis = LaunchConfiguration('chassis', default='diff')
    gui = LaunchConfiguration('gui', default='true')
    xacro_file = PythonExpression([
        "'", diff_xacro, "' if '", chassis, "' == 'diff' else '", mecanum_xacro, "'"
    ])
    use_gazebo_odom_tf_xacro = PythonExpression([
        "'true' if '", use_gazebo_odom_tf, "' in ('true', 'True') else 'false'"
    ])
    robot_description = Command([
        'xacro ', xacro_file, ' use_gazebo_odom_tf:=', use_gazebo_odom_tf_xacro
    ])

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether to enable SLAM mode'
    )
    use_gazebo_odom_tf_arg = DeclareLaunchArgument(
        'use_gazebo_odom_tf',
        default_value='false',
        description=(
            'Publish Gazebo odom TF from planar_move / diff_drive. Keep false when '
            'an external odometry source such as BIEVR-LIO publishes odom -> base_footprint.'
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
    chassis_arg = DeclareLaunchArgument(
        'chassis',
        default_value='diff',
        description='Chassis kinematics: diff (diff_drive, default) or mecanum (planar_move)'
    )
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run gzclient. Set false for headless gzserver-only tests.'
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': True,
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': empty_world,
            'gui': gui,
            'verbose': 'true',
            'pause': 'false',
        }.items(),
    )

    robot_spawn_x = PythonExpression([
        "'-11.8' if '", scene, "' == 'culvert' else '0.0'"
    ])

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_cool_robot',
            '-x', robot_spawn_x,
            '-y', '0.0',
            '-z', '0.05',
            '-Y', '0.0',
            '-timeout', '60.0',
        ],
        output='screen'
    )

    spawn_culvert_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', culvert_file,
            '-entity', 'culvert',
            '-timeout', '60.0',
        ],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", scene, "' == 'culvert' and '",
            spawn_culvert, "' == 'true'"
        ]))
    )

    spawn_map1_obstacles = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', map1_obstacles_file,
            '-entity', 'map1_obstacles',
            '-timeout', '60.0',
        ],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", scene, "' == 'map1' and '",
            spawn_map1_obstacles, "' == 'true'"
        ]))
    )

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
        chassis_arg,
        gui_arg,
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=os.environ['GAZEBO_MODEL_PATH']),
        SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=os.environ['GAZEBO_PLUGIN_PATH']),
        SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value=''),
        gazebo,
        spawn_entity,
        spawn_culvert_node,
        spawn_map1_obstacles,
        pointcloud_to_scan,
        simulated_imu,
        node_robot_state_publisher,
    ])
