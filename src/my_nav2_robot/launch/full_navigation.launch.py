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
    # nav2_bringup 内部用 PythonExpression 直接拼接 slam 字符串，
    # 所以这里统一转成 Python 布尔字面量，兼容 slam:=true 和 slam:=True。
    slam_for_bringup = PythonExpression([
        "'True' if '", slam_mode, "' in ('true', 'True') else 'False'"
    ])

    nav2_params_slam = os.path.join(pkg_project_bringup, 'config', 'nav2_params_slam.yaml')
    nav2_params_nav = os.path.join(pkg_project_bringup, 'config', 'nav2_params_nav_diy.yaml')

    # 部分变量定义
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'true')

    # 静态场景选择，与 gazebo_sim.launch.py 的 scene 参数保持一致。
    # 默认使用涵洞场景；地图默认为 maps/<scene>.yaml，可用 map:= 显式覆盖。
    scene = LaunchConfiguration('scene', default='culvert')
    spawn_culvert = LaunchConfiguration('spawn_culvert', default='true')
    spawn_map1_obstacles = LaunchConfiguration('spawn_map1_obstacles', default='true')
    enable_tunnel_guidance = LaunchConfiguration('enable_tunnel_guidance', default='false')
    map_override = LaunchConfiguration('map', default='')

    # AMCL 初始位姿跟随机器人出生点：
    #   culvert -> 左侧入口 x=-5.8
    #   map1    -> 地图中心 (0, 0)
    amcl_initial_pose = PythonExpression([
        "'[-5.8, 0.0, 0.0]' if '",
        scene, "' == 'culvert' else '[0.0, 0.0, 0.0]'"
    ])
    nav_params_with_pose = RewrittenYaml(
        source_file=nav2_params_nav,
        param_rewrites={
            'amcl.ros__parameters.initial_pose': amcl_initial_pose,
        },
        convert_types=True,
    )
    map_yaml_file = PythonExpression([
        "'", os.path.join(pkg_project_bringup, 'maps'), '/', scene, ".yaml' if '",
        map_override, "' == '' else '",
        map_override, "'"
    ])

    # gz找模型路径
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_project_bringup, '..')] # 指向 install/my_nav2_robot/share 这一层
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
            'nav2_params': nav2_params_slam,
            'use_sim_time': use_sim_time,
            'slam': slam_for_bringup,
            'scene': scene,
            'spawn_culvert': spawn_culvert,
            'spawn_map1_obstacles': spawn_map1_obstacles
        }.items()
    )
    gazebo_sim_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'gazebo_sim.launch.py')
        ),
        condition=UnlessCondition(slam_mode),
        launch_arguments={
            'nav2_params': nav2_params_nav,
            'use_sim_time': use_sim_time,
            'slam': slam_for_bringup,
            'scene': scene,
            'spawn_culvert': spawn_culvert,
            'spawn_map1_obstacles': spawn_map1_obstacles
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
            'params_file': nav2_params_slam,
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

    # 涵洞中心线估计节点：默认关闭，独立调试时可通过 launch 文件单独启动。
    tunnel_guidance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_tunnel_guidance'),
                'launch', 'tunnel_guidance.launch.py')
        ),
        condition=IfCondition(enable_tunnel_guidance)
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
            'enable_tunnel_guidance',
            default_value='false',
            description='Start the tunnel centerline guidance node'),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Explicit map YAML path; empty uses maps/<scene>.yaml'),
        gazebo_sim_slam,
        gazebo_sim_nav,
        static_tf_node,
        nav2_bringup_launch_slam,
        nav2_bringup_launch_nav,
        tunnel_guidance_launch,
        rviz_node
    ])
