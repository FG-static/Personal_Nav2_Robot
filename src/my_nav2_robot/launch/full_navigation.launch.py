import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # 地图路径
    slam_mode = LaunchConfiguration('slam', default='False')

    nav2_params_slam = os.path.join(pkg_project_bringup, 'config', 'nav2_params_slam.yaml')
    nav2_params_nav = os.path.join(pkg_project_bringup, 'config', 'nav2_params_nav_diy.yaml')

    # 部分变量定义
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'true')

    # 静态场景选择，与 gazebo_sim.launch.py 的 scene 参数保持一致。
    # 默认使用涵洞场景；地图默认为 maps/<scene>.yaml，可用 map:= 显式覆盖。
    scene = LaunchConfiguration('scene', default='culvert')
    spawn_culvert = LaunchConfiguration('spawn_culvert', default='true')
    spawn_map1_obstacles = LaunchConfiguration('spawn_map1_obstacles', default='true')
    map_override = LaunchConfiguration('map', default='')
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
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0', 
                '--yaw', '0', '--pitch', '0', '--roll', '0', 
                '--frame-id', 'map', 
                '--child-frame-id', 'odom']
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
            'scene': scene,
            'spawn_culvert': spawn_culvert,
            'spawn_map1_obstacles': spawn_map1_obstacles
        }.items()
    )

    # slam建图模式
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        condition=IfCondition(slam_mode),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

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
            'map': dynamic_map_path,
            'params_file': nav2_params_nav,
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
        gazebo_sim_slam,
        gazebo_sim_nav,
        static_tf_node,
        slam_toolbox,
        nav2_bringup_launch_slam,
        nav2_bringup_launch_nav,
        rviz_node
    ])