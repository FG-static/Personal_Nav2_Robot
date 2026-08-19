import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from nav2_common.launch import RewrittenYaml
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    # 包地址
    pkg_project_bringup = get_package_share_directory('my_nav2_robot')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 地图路径
    slam_mode = LaunchConfiguration('slam', default='False')

    # 使用 BIEVR-LIO 等外部里程计时必须关闭 Gazebo 的动态 odom TF，
    # 避免两个节点同时发布 odom -> base_footprint。
    use_gazebo_odom_tf = LaunchConfiguration('use_gazebo_odom_tf', default='false')
    # nav2_bringup 内部用 PythonExpression 直接拼接 slam 字符串，
    # 所以这里统一转成 Python 布尔字面量，兼容 slam:=true 和 slam:=True。
    slam_for_bringup = PythonExpression([
        "'True' if '", slam_mode, "' in ('true', 'True') else 'False'"
    ])

    chassis = LaunchConfiguration('chassis', default='mecanum')
    gui = LaunchConfiguration('gui', default='true')
    nav2_params_slam_mecanum = os.path.join(
        pkg_project_bringup, 'config', 'nav2_params_slam.yaml')
    nav2_params_slam_diff = os.path.join(
        pkg_project_bringup, 'config', 'nav2_params_slam_diff.yaml')
    nav2_params_slam = PythonExpression([
        "'", nav2_params_slam_diff, "' if '", chassis,
        "' == 'diff' else '", nav2_params_slam_mecanum, "'"
    ])
    nav2_params_nav = os.path.join(pkg_project_bringup, 'config', 'nav2_params_nav_diy.yaml')

    # 部分变量定义
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'true')

    # 静态场景选择，与 gazebo_sim.launch.py 的 scene 参数保持一致。
    # 默认使用涵洞场景；地图默认为 maps/<scene>.yaml，可用 map:= 显式覆盖。
    scene = LaunchConfiguration('scene', default='culvert')
    spawn_culvert = LaunchConfiguration('spawn_culvert', default='true')
    spawn_map1_obstacles = LaunchConfiguration('spawn_map1_obstacles', default='true')
    map_override = LaunchConfiguration('map', default='')

    # AMCL nested initial_pose (Humble) follows spawn:
    #   culvert -> 左侧入口 x=-11.8
    #   map1    -> 地图中心 (0, 0)
    amcl_initial_x = PythonExpression([
        "'-11.8' if '", scene, "' == 'culvert' else '0.0'"
    ])
    bt_xml_mecanum = os.path.join(pkg_project_bringup, 'behavior_trees', 'test_nav.xml')
    bt_xml_diff = os.path.join(pkg_project_bringup, 'behavior_trees', 'test_nav_diff.xml')
    default_nav_to_pose_bt_xml = PythonExpression([
        "'", bt_xml_diff, "' if '", chassis,
        "' == 'diff' else '", bt_xml_mecanum, "'"
    ])
    nav_params_with_pose = RewrittenYaml(
        source_file=nav2_params_nav,
        param_rewrites={
            'amcl.ros__parameters.initial_pose.x': amcl_initial_x,
            'amcl.ros__parameters.initial_pose.y': '0.0',
            'amcl.ros__parameters.initial_pose.z': '0.0',
            'amcl.ros__parameters.initial_pose.yaw': '0.0',
            'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        },
        convert_types=True,
    )
    slam_params_rewritten = RewrittenYaml(
        source_file=nav2_params_slam,
        param_rewrites={
            'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        },
        convert_types=True,
    )
    map_yaml_file = PythonExpression([
        "'", os.path.join(pkg_project_bringup, 'maps'), '/', scene, ".yaml' if '",
        map_override, "' == '' else '",
        map_override, "'"
    ])

    # Gazebo Classic model path: parent of models/culvert.sdf and models/map1_obstacles.
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_project_bringup, 'models')
    )

    # map->odom
    # 导航模式使用外部 LIO，手动发布静态 map->odom；
    # SLAM 模式下该 TF 由 slam_toolbox 发布，不能同时手动发布，否则会冲突。
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom'],
        condition=UnlessCondition(slam_mode)
    )

    # gazebo_sim.launch.py
    gazebo_sim_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'gazebo_sim.launch.py')
        ),
        condition=IfCondition(slam_mode),
        launch_arguments={
            'nav2_params': slam_params_rewritten,
            'use_sim_time': use_sim_time,
            'slam': slam_for_bringup,
            'use_gazebo_odom_tf': use_gazebo_odom_tf,
            'scene': scene,
            'spawn_culvert': spawn_culvert,
            'spawn_map1_obstacles': spawn_map1_obstacles,
            'chassis': chassis,
            'gui': gui,
        }.items()
    )
    gazebo_sim_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'gazebo_sim.launch.py')
        ),
        condition=UnlessCondition(slam_mode),
        launch_arguments={
            'nav2_params': nav_params_with_pose,
            'use_sim_time': use_sim_time,
            'slam': slam_for_bringup,
            'use_gazebo_odom_tf': use_gazebo_odom_tf,
            'scene': scene,
            'spawn_culvert': spawn_culvert,
            'spawn_map1_obstacles': spawn_map1_obstacles,
            'chassis': chassis,
            'gui': gui,
        }.items()
    )

    # SLAM 功能由 nav2_bringup 的 slam_launch.py 统一启动，
    # 这里不再单独启动 slam_toolbox，避免同时跑两套 SLAM。
    # bringup_launch.py
    '''
    dynamic_map_path = PythonExpression([ # nav下不加载地图
        "'' if ", slam_mode, " == 'True' else '", map_yaml_file, "'"
    ])
    '''
    dynamic_map_path = PythonExpression([
        "'' if '", slam_mode, "' == 'True' else '",
        map_yaml_file, "'"
    ])
    nav2_bringup_launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        condition=IfCondition(slam_mode),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': slam_for_bringup,
            'map': dynamic_map_path,
            'params_file': slam_params_rewritten,
        }.items()
    )
    nav2_bringup_launch_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        condition=UnlessCondition(slam_mode),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': slam_for_bringup,
            'map': dynamic_map_path,
            'params_file': nav_params_with_pose,
        }.items()
    )

    rviz_config_path = os.path.join(pkg_project_bringup, 'config', 'nav2_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='False', description='Whether to run SLAM'),
        DeclareLaunchArgument(
            'use_gazebo_odom_tf',
            default_value='false',
            description=(
                'Use Gazebo ground-truth odom TF instead of an external odometry '
                'source such as BIEVR-LIO.'
            )
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'scene',
            default_value='culvert',
            description='Static scene and map name: culvert or map1'),
        DeclareLaunchArgument(
            'spawn_culvert',
            default_value='true',
            description='Spawn the culvert walls when scene is culvert'),
        DeclareLaunchArgument(
            'spawn_map1_obstacles',
            default_value='true',
            description='Spawn map1 obstacles when scene is map1'),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Explicit map YAML path; empty uses maps/<scene>.yaml'),
        DeclareLaunchArgument(
            'chassis',
            default_value='mecanum',
            description=(
                'Chassis kinematics: mecanum (default) or diff. '
                'diff uses robot_diff.urdf.xacro; with slam:=true also uses '
                'nav2_params_slam_diff.yaml and DWA.'
            )),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Run Gazebo Classic gzclient. Set false for headless tests.'),
        set_gazebo_model_path,
        gazebo_sim_slam,
        gazebo_sim_nav,
        static_tf_node,
        nav2_bringup_launch_slam,
        nav2_bringup_launch_nav,
        rviz_node
    ])
