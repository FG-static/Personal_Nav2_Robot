# ROS 2 Humble 移植方案（方案 A / 方案 B）

本文记录把本仓库从 **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic（gz-sim8）** 迁到 **Ubuntu 22.04 + ROS 2 Humble** 的两套完整方案。

- 只在 `humble` 分支实施，`main` 继续给 Jazzy / 24.04。
- 方案 A、B **共用同一套 Nav2 / 行为树 / yaml 改动**，差别只在仿真层（Gazebo Classic vs Ignition Fortress）。
- 写本文时的本机事实：Ubuntu 22.04.5、ROS Humble 已 source、Gazebo Classic 11.10.2 与 `gazebo_ros_pkgs` 已装、**未装** `ros-humble-ros-gz` / Ignition Gazebo 6。

相关入口：

- 项目说明：[`README.md`](README.md)
- Mid360 CSV 来源：[`my_nav2_robot/config/MID360_SCAN_PATTERN.md`](my_nav2_robot/config/MID360_SCAN_PATTERN.md)

---

## 1. 现状对照

### 1.1 发行版与仿真

| 项 | 上游 `main`（Jazzy） | 本机 Humble | 方案 A 目标 | 方案 B 目标 |
| --- | --- | --- | --- | --- |
| 系统 | Ubuntu 24.04 | Ubuntu 22.04.5 | 不变 | 不变 |
| ROS | Jazzy | Humble | Humble | Humble |
| Python（ROS 要求） | 3.12 | 必须用 `/usr/bin/python3`（3.10） | 同左 | 同左 |
| 仿真 | Gazebo Harmonic `gz-sim8` | Classic 11 已装，Fortress 未装 | **Gazebo Classic 11** + `gazebo_ros` | **Ignition Fortress（gz-sim6）** + `ros_gz` |
| 桥接 | `ros_gz_bridge` / `ros_gz_sim` | 无 ros-gz | 不需要 transport bridge，插件直接发 ROS 话题 | 继续用 `ros_gz_bridge`，消息类型改为 Fortress |
| 自定义雷达 | `libmy_nav2_robot_mid360_gz_plugin.so`（`gz::sim::System`） | 无法链接 gz-sim8 | 改写成 Gazebo 11 `ModelPlugin` | 改写成 `ignition::gazebo::System` |
| 麦轮驱动 | `gz-sim-velocity-control-system` | — | `libgazebo_ros_planar_move.so` | Fortress `VelocityControl`（`ignition::gazebo::systems`） |
| 差速驱动 | `gz-sim-diff-drive-system` | — | `libgazebo_ros_diff_drive.so` | Fortress `DiffDrive` |
| 关节 / 真值里程计 | `JointStatePublisher` / `OdometryPublisher` | — | `joint_state_publisher` + `p3d` | 对应 Fortress system plugin |
| 世界文件 | launch 里 `sensors.sdf`（仓库中不存在） | — | `empty.world` 或自备 Classic world | `empty.sdf` 或把场景写成 Fortress sdf |

### 1.2 本机已装 / 未装（2026-08-19 核对）

已装、可直接用：

- `ros-humble-nav2-*`（含 MPPI、collision-monitor、bringup）
- `ros-humble-slam-toolbox`
- `ros-humble-gazebo-ros-pkgs` 以及 `libgazebo_ros_planar_move.so`、`diff_drive`、`p3d`、`joint_state_publisher`、`ray_sensor` 等
- `ros-humble-behaviortree-cpp-v3`（3.8.7）
- `ros-humble-libg2o`、Eigen、OpenCV、PCL、`cv_bridge`、`pcl_ros`
- Sophus：`/usr/local/share/sophus/cmake/SophusConfig.cmake`
- AMCL Omni：`/opt/ros/humble/include/nav2_amcl/motion_model/omni_motion_model.hpp`

未装、两套方案都要用 apt 补的：

```bash
sudo apt install ros-humble-osqp-vendor
```

仅方案 B 额外需要：

```bash
sudo apt install ros-humble-ros-gz
# 会拉上 libignition-gazebo6-dev、ros-humble-ros-gz-sim、ros-humble-ros-gz-bridge 等
```

没有、且不挡主路径（第一轮跳过）：

- `small_gicp`（`nav2_visual_processing` 依赖）
- `opennav_docking` / `nav2_docking`（Humble 发行版无对接服务器）

### 1.3 必须先处理的本机 Python 冲突

当前 `PATH` 上的 `python3` 是 **Miniconda Python 3.13**。ROS Humble 的 `launch`、`PyYAML`、`rclpy` 都装在系统 Python 3.10 上。直接 `ros2 launch` 会出现 `No module named 'yaml'`。

编译和启动前任选其一：

```bash
conda deactivate
# 或
export PATH="/usr/bin:$PATH"
```

确认：

```bash
which python3          # 应为 /usr/bin/python3
python3 --version      # 3.10.x
python3 -c "import yaml, launch; print('ok')"
```

---

## 2. 两套方案共用的改动（先做，与 A/B 无关）

下面这些不依赖选 Classic 还是 Fortress。建议先合进 `humble`，再分叉仿真层。

### 2.1 全局规划器：`createPlan` 去掉 `cancel_checker`

Humble `nav2_core::GlobalPlanner::createPlan` 只有 `(start, goal)`。Jazzy 多了第三个参数 `std::function<bool()> cancel_checker`。现在带 `override` 的声明在 Humble 上会编译失败。

改这三个插件的头文件和实现，函数体里原本就没使用该参数：

- `my_nav2_planner/include/my_nav2_planner/astar_planner.hpp`
- `my_nav2_planner/src/astar_planner.cpp`
- `my_rrtstar_planner/include/my_rrtstar_planner/rrtstar_planner.hpp`
- `my_rrtstar_planner/src/rrtstar_planner.cpp`
- `my_hybrid_astar_planner/include/my_hybrid_astar_planner/hybrid_astar_planner.hpp`
- `my_hybrid_astar_planner/src/hybrid_astar_planner.cpp`

Humble 目标签名：

```cpp
nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) override;
```

`configure(WeakPtr, name, tf, costmap_ros)` 与 Humble 一致，不用改。

### 2.2 异常头文件

| Jazzy | Humble |
| --- | --- |
| `nav2_core/planner_exceptions.hpp` | `nav2_core/exceptions.hpp` |
| `nav2_core/controller_exceptions.hpp` | `nav2_core/exceptions.hpp` |

类名仍是 `nav2_core::PlannerException` / `nav2_core::ControllerException`。涉及：

- `astar_planner.cpp`
- `rrtstar_planner.cpp`
- `hybrid_astar_planner.cpp`
- `dwa_controller.cpp`
- `mpc_controller.cpp`
- `teb_controller.cpp`

### 2.3 行为树：BT.CPP v4 → v3

Humble Nav2 官方头文件用的是 `#include "behaviortree_cpp_v3/action_node.h"`。

**CMake / package.xml（`my_nav2_bt_nodes`）**

- 依赖名：`behaviortree_cpp` → `behaviortree_cpp_v3`
- `find_package(behaviortree_cpp_v3 REQUIRED)`
- `ament_target_dependencies` 同步改

**C++（`manage_trajectory_action.hpp/.cpp`）**

```cpp
// 旧（Jazzy）
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/json_export.h"
#include "nav2_behavior_tree/json_utils.hpp"

// 新（Humble）
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/bt_conversions.hpp"
```

删除 `BT::RegisterJsonDefinition<...>()`。Humble 没有 `json_utils.hpp`。`PoseStamped` / `Path` 走 `bt_conversions.hpp` 即可。`BT_REGISTER_NODES(factory)` 这段 **保留**（Humble 仍然靠它加载自定义节点）。

`create_callback_group(..., false)` 在 Humble rclcpp 里存在，独立 executor 线程的写法可以保留。

**XML（`test_nav.xml`、`test_nav_diff.xml`）**

Jazzy：

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
```

Humble（对照 `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/`）：

```xml
<root main_tree_to_execute="MainTree">
```

节点标签（`RecoveryNode`、`PipelineSequence`、`RateController`、`ComputePathToPose`、`SmoothPath`、`FollowPath`、`ManageTrajectory`、`GoalUpdated`、`BackUp`、`ClearEntireCostmap`、`Wait`）Humble 都有对应插件，逻辑树本身可保持。`<TreeNodesModel>` 可留着给 Groot，不影响运行。

### 2.4 `bt_navigator` yaml：必须列出全部官方 BT 插件

Jazzy 内置节点自动加载，自定义 yaml 里只有：

```yaml
plugin_lib_names:
  - my_manage_trajectory_bt_node
navigators: ["navigate_to_pose", "navigate_through_poses"]
navigate_to_pose:
  plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
```

Humble **没有** `navigators:` 插件化接口，以上字段要删掉。`plugin_lib_names` 必须包含官方完整列表（见 `/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml`），**再追加**：

```yaml
      - my_manage_trajectory_bt_node
```

同时建议加上 Humble 里常见的两个辅助 node 段（否则部分发行版会抱怨缺参数）：

```yaml
bt_navigator_navigate_through_poses_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator_navigate_to_pose_rclcpp_node:
  ros__parameters:
    use_sim_time: True
```

`default_nav_to_pose_bt_xml` 不要写死 `/home/goose/nav2_test/...`。在 `full_navigation.launch.py` 里用 `RewrittenYaml` 改成：

```python
os.path.join(pkg_project_bringup, 'behavior_trees', 'test_nav.xml')
# chassis:=diff 时用 test_nav_diff.xml
```

### 2.5 其余 Nav2 yaml（四份都要改）

文件：

- `my_nav2_robot/config/nav2_params_nav_diy.yaml`
- `my_nav2_robot/config/nav2_params_nav.yaml`
- `my_nav2_robot/config/nav2_params_slam.yaml`
- `my_nav2_robot/config/nav2_params_slam_diff.yaml`

Humble `navigation_launch.py` 的 lifecycle 节点只有：

`controller_server`、`smoother_server`、`planner_server`、`behavior_server`、`bt_navigator`、`waypoint_follower`、`velocity_smoother`。

对照修改：

| Jazzy 字段 | Humble 处理 |
| --- | --- |
| `progress_checker_plugins: [...]` | 改成单数 `progress_checker_plugin: "progress_checker"` |
| `enable_stamped_cmd_vel` | 删除（controller / behavior / velocity_smoother 三段都有） |
| `action_server_result_timeout` | 删除 |
| `error_code_name_prefixes` | 删除 |
| `costmap_update_timeout` | 删除 |
| `current_goal_checker` / `current_progress_checker` | Humble 用 `goal_checker_plugins` + `progress_checker_plugin`，按官方 yaml 对齐 |
| AMCL `initial_pose: [x, y, yaw]` | 改为 `set_initial_pose: True` + 嵌套 `initial_pose.{x,y,z,yaw}` |
| `docking_server` 整段 | **删除**（Humble 无此包） |
| `collision_monitor` 整段 | 可删。Humble 有该包，但官方 bringup **不会启动它** |
| costmap VoxelLayer / ObstacleLayer / InflationLayer | 保留，Humble 有 |
| MPPI critics 名称 | 与 Humble `critics.xml` 一致（ConstraintCritic、CostCritic 等），保留 |
| `robot_model_type: nav2_amcl::OmniMotionModel` | Humble 有头文件，麦轮配置可保留 |

AMCL 初值示例（涵洞入口）：

```yaml
amcl:
  ros__parameters:
    set_initial_pose: True
    initial_pose:
      x: -11.8
      y: 0.0
      z: 0.0
      yaw: 0.0
```

`full_navigation.launch.py` 里现在的 RewrittenYaml 键 `amcl.ros__parameters.initial_pose` 对 Humble 无效，应改成分别重写 `x` / `y` / `yaw`，或按 `scene` 准备两份 yaml。

### 2.6 文档与可选包

- `src/README.md`：环境改为 Ubuntu 22.04 + Humble；`ros-jazzy-pointcloud-to-laserscan` 改为 `ros-humble-pointcloud-to-laserscan`。
- `nav2_visual_processing`：依赖 `small_gicp`，本机没有，也不在 `full_navigation.launch.py` 里。第一轮用 `COLCON_IGNORE` 或从工作区移走，不挡主路径。
- `my_tunnel_guidance`：纯 ROS + PCL，无 gz-sim8。第一轮能编过即可，不作为仿真验收门槛。
- `my_planning_metrics`、`rm_interfaces`：与发行版无关，保持。

### 2.7 共用验收（不启动 Gazebo 也能做）

```bash
conda deactivate
source /opt/ros/humble/setup.bash
cd /path/to/nav2_test
colcon build --symlink-install --packages-skip nav2_visual_processing
source install/setup.bash
```

规划器 / 控制器 / 平滑器 / BT 节点应全部编过。仿真相关目标在方案 A/B 里再分别编。

---

## 3. 方案 A：Gazebo Classic 11 + `gazebo_ros`

### 3.1 思路

跟本机已装软件对齐：不再使用 `gz-sim` transport，也不装 Fortress。URDF 里的驱动、关节、真值里程计换成 `gazebo_ros` 插件（插件自己发布 ROS 2 话题）。Mid360 自定义插件改成 Gazebo 11 的 `gazebo::ModelPlugin`，继续向 `/livox/lidar` 发 `sensor_msgs/PointCloud2`，后面的 `pointcloud_to_laserscan`、IMU 伪造节点、Nav2 都不用改话题名。

**优点**

- 零额外仿真发行版，sudo 只装 `osqp-vendor`（`/scan` 由本仓库点云切片节点生成）。
- 本机已经有全套 `libgazebo_ros_*.so`。
- 不依赖 `parameter_bridge` 的 `gz.msgs` / `ignition.msgs` 类型字符串。

**缺点**

- Mid360 插件要从 Harmonic ECM 重写到 Classic 物理引擎，是本方案最大工作量。
- `planar_move` 是平面运动学，和 Harmonic `VelocityControl` 手感会差一点，控制器参数可能要微调。
- Classic `diff_drive` 通常左右各一个关节；现在 URDF 是同侧前后轮并联，可能只能驱动前轮或改成 skid-steer 折中。
- SDF 1.10 的涵洞 / map1 模型要降到 1.6/1.7。

### 3.2 依赖改动（`my_nav2_robot`）

`package.xml` 删除：

```xml
<depend>gz_sim_vendor</depend>
<depend>gz_plugin_vendor</depend>
<exec_depend>ros_gz_bridge</exec_depend>
<exec_depend>ros_gz_sim</exec_depend>
```

改为：

```xml
<depend>gazebo_dev</depend>
<depend>gazebo_ros</depend>
<exec_depend>gazebo_ros_pkgs</exec_depend>
<exec_depend>xacro</exec_depend>
```

`CMakeLists.txt` 删除：

```cmake
find_package(gz-sim8 REQUIRED HINTS ...)
find_package(gz-plugin2 REQUIRED HINTS ...)
target_link_libraries(my_nav2_robot_mid360_gz_plugin gz-sim8::core gz-plugin2::register)
```

改为类似：

```cmake
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
# Mid360 Classic 插件
target_link_libraries(my_nav2_robot_mid360_gz_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(my_nav2_robot_mid360_gz_plugin rclcpp sensor_msgs gazebo_ros gazebo_dev)
```

插件文件名可继续叫 `libmy_nav2_robot_mid360_gz_plugin.so`（少改 URDF），或改名为 `libmy_nav2_robot_mid360_gazebo_plugin.so` 并同步 URDF。

`env-hooks/my_nav2_robot.dsv.in` 现在是：

```
prepend-non-duplicate;GZ_SIM_SYSTEM_PLUGIN_PATH;lib
```

方案 A 改为：

```
prepend-non-duplicate;GAZEBO_PLUGIN_PATH;lib
```

`full_navigation.launch.py` 里的 `GZ_SIM_RESOURCE_PATH` 改为 `GAZEBO_MODEL_PATH`，指向 `share/my_nav2_robot` 的上一级（`share`），让 `models/culvert.sdf`、`models/map1_obstacles` 能被找到。

### 3.3 launch 重写（`gazebo_sim.launch.py`）

当前关键调用：

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
    launch_arguments={'gz_args': '-r sensors.sdf'}.items(),
)
# spawn：ros_gz_sim 的 executable='create'
# 桥：ros_gz_bridge parameter_bridge
```

方案 A 替换为：

1. 启动 Classic：

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
    launch_arguments={
        'world': os.path.join(
            get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world'),  # 或自备 world
        'verbose': 'true',
    }.items(),
)
```

2. 生成机器人：

```python
Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-topic', 'robot_description',
        '-entity', 'my_cool_robot',
        '-x', robot_spawn_x, '-y', '0.0', '-z', '0.05',
        '-Y', '0.0',
    ],
    output='screen',
)
```

3. 静态场景：Classic 可用第二个 `spawn_entity.py -file <sdf> -entity culvert`，或把涵洞做成 world 的一部分。`ros_gz_sim create -world sensors` 这条路径删掉。

4. **整段 `parameter_bridge` / `tf_bridge` 删除。** Classic 插件直接发 `/cmd_vel` 订阅、`/joint_states`、`/ground_truth`。`/clock` 由 `gazebo_ros_init` 提供（`use_sim_time:=true`）。

5. 保留：

- `robot_state_publisher`
- `pointcloud_to_laserscan.launch.py`（`/livox/lidar` → `/scan`）
- `simulated_imu_publisher`（`/ground_truth` → `/livox/imu`）

`slam` + `use_gazebo_odom_tf` 为真时：让 `planar_move` / `p3d` 发布 `odom → base_footprint`。为假时关闭插件的 `publish_odom_tf`，把 TF 留给 BIEVR-LIO。这对应现在「不桥接 gz `/tf`」的语义。

### 3.4 URDF 插件对照（`robot.urdf.xacro` / `robot_diff.urdf.xacro`）

**麦轮（当前 VelocityControl）→ planar_move**

```xml
<gazebo>
  <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
    <ros>
      <remapping>cmd_vel:=cmd_vel</remapping>
      <remapping>odom:=wheel_odom</remapping>
    </ros>
    <update_rate>100</update_rate>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>false</publish_odom_tf>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_footprint</robot_base_frame>
  </plugin>
</gazebo>
```

`publish_odom_tf` 是否打开，要和 launch 的 `use_gazebo_odom_tf` 一致。xacro 可用 `$(arg ...)` 切。

**差速（当前 DiffDrive）→ gazebo_ros_diff_drive**

```xml
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <ros>
      <remapping>cmd_vel:=cmd_vel</remapping>
      <remapping>odom:=wheel_odom</remapping>
    </ros>
    <num_wheel_pairs>1</num_wheel_pairs>
    <left_joint>front_left_wheel_joint</left_joint>
    <right_joint>front_right_wheel_joint</right_joint>
    <wheel_separation>${wheel_separation}</wheel_separation>
    <wheel_diameter>${2 * wheel_radius}</wheel_diameter>
    <max_wheel_torque>50</max_wheel_torque>
    <max_wheel_acceleration>3.0</max_wheel_acceleration>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>false</publish_odom_tf>
    <publish_wheel_tf>false</publish_wheel_tf>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_footprint</robot_base_frame>
  </plugin>
</gazebo>
```

后轮若无驱动会拖着转。若仿真打滑明显，再评估是否加第二对轮或改成 skid-steer。

**关节状态**

```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <ros>
      <remapping>joint_states:=joint_states</remapping>
    </ros>
    <update_rate>50</update_rate>
  </plugin>
</gazebo>
```

**真值里程计（替代 OdometryPublisher → `/ground_truth`）**

```xml
<gazebo>
  <plugin name="p3d_ground_truth" filename="libgazebo_ros_p3d.so">
    <ros>
      <remapping>odom:=ground_truth</remapping>
    </ros>
    <body_name>base_link</body_name>
    <frame_name>world</frame_name>
    <update_rate>200.0</update_rate>
    <xyz_offset>0 0 0</xyz_offset>
    <rpy_offset>0 0 0</rpy_offset>
  </plugin>
</gazebo>
```

`body_name` 必须是 URDF 里真实存在、且在 Gazebo 中有物理 body 的 link。若 `base_footprint` 无惯性，用 `base_link`。`simulated_imu_publisher` 继续订 `/ground_truth`。

**删除的 gz 插件块**

- `gz-sim-velocity-control-system`
- `gz-sim-diff-drive-system`
- `gz-sim-joint-state-publisher-system`
- `gz-sim-odometry-publisher-system`
- 已注释的 `gz-sim-mecanum-drive-system`、`gz-sim-diff-drive-system` 备份块（避免以后误开）
- 深度相机注释里的 `<gz_frame_id>`（若恢复相机，改用 `libgazebo_ros_camera.so`）

**Mid360 插件在 URDF 中的挂法（Classic）**

```xml
<gazebo>
  <plugin name="mid360" filename="libmy_nav2_robot_mid360_gz_plugin.so">
    <tracking_link_name>livox_frame</tracking_link_name>
    <frame_id>livox_frame</frame_id>
    <topic>/livox/lidar</topic>
    <csv_file_name>$(find my_nav2_robot)/config/mid360.csv</csv_file_name>
    <!-- 其余 samples_per_scan / update_rate / min_range / max_range 等保持 -->
  </plugin>
</gazebo>
```

Classic 用 `$(find pkg)` 或在插件 `Load()` 里用 `ament_index_cpp` 解析 share 路径。不要依赖 Harmonic 的 SDF 元素遍历方式。

### 3.5 Mid360 插件重写要点（方案 A 核心）

现实现：`include/my_nav2_robot/mid360_gz_plugin.hpp` + `src/mid360_gz_plugin.cpp`。

它做的事情（算法可复用）：

1. 读 `mid360.csv` 得到每根射线方向。
2. 每周期从 `livox_frame` 位姿发出 `samples_per_scan` 根射线。
3. 与世界里的 box / plane / sphere / cylinder collision 求交。
4. 加距离噪声、dropout，拼成 `PointCloud2` 发到 `/livox/lidar`。

Harmonic 专用、必须换掉的部分：

| Harmonic | Classic 11 |
| --- | --- |
| `gz::sim::System` + `ISystemConfigure` + `ISystemPostUpdate` | `gazebo::ModelPlugin`，`Load` + `OnUpdate` |
| `gz::sim::EntityComponentManager` 扫 collision | `gazebo::physics::World` / `Model` / `Link` / `Collision` 的 shape |
| `gz::math::Pose3d` / `Vector3d` | `ignition::math`（Gazebo 11 已用 ignition-math6）或 `gazebo::math` 遗留类型 |
| `GZ_ADD_PLUGIN` / `gz-plugin2` | `GZ_REGISTER_MODEL_PLUGIN(Mid360GazeboPlugin)` |
| 自己实现的 box/sphere 求交 | 可继续自写；或改用 `gazebo::physics::RayShape` / `engine->CreateShape("ray")` |

建议结构：

```cpp
class Mid360GazeboPlugin : public gazebo::ModelPlugin {
 public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
 private:
  void OnUpdate(const gazebo::common::UpdateInfo &info);
  gazebo::event::ConnectionPtr update_connection_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};
```

`Load()` 里 `rclcpp::init` 若已被 `gazebo_ros` 做过，则只创建 node，不要二次 init。可参考 `gazebo_ros` 插件如何拿 ROS 节点（`gazebo_ros::Node::Get()`）。

CSV 解析、噪声、拼 PointCloud2 的代码尽量原样搬，避免 LIO 时间戳格式变掉。字段约定见 `MID360_SCAN_PATTERN.md`。

### 3.6 世界与 SDF

- launch 使用的 `sensors.sdf` 不在仓库里，方案 A 不要再引用它。
- `models/culvert.sdf`、`models/map1_obstacles/model.sdf` 的 `sdf version="1.10"` 改为 `"1.7"`（Classic 11 常用 1.6/1.7）。
- 检查是否用了 Harmonic 才有的标签（例如某些 `material` / `pbr`）。没有的话只改 version 往往就能加载。
- 模型目录保留 `model.config`，并把 `GAZEBO_MODEL_PATH` 指到 `share`。

地面：`empty.world` 自带地面平面。涵洞「不封底」时车轮需要这个地。

### 3.7 方案 A 话题契约（尽量与现在一致）

| 话题 | 方案 A 来源 |
| --- | --- |
| `/clock` | `gazebo_ros` |
| `/cmd_vel` | planar_move 或 diff_drive 订阅 |
| `/joint_states` | `gazebo_ros_joint_state_publisher` |
| `/ground_truth` | `gazebo_ros_p3d` |
| `/livox/lidar` | 自定义 Mid360 Classic 插件 |
| `/livox/imu` | `simulated_imu_publisher` |
| `/scan` | `my_nav2_robot/pointcloud_to_laserscan_node`（Mid360 点云水平切片） |
| `odom → base_footprint` | 仅当 `use_gazebo_odom_tf:=true` 时由驱动/p3d 发布 |

不再出现 `/camera/*`、`/depth_camera/*` 桥（那些传感器在 xacro 里已注释）。

### 3.8 方案 A 实施顺序

1. 完成第 2 节共用改动，编译跳过仿真插件，确认规划器/BT 能过。
2. 改 `package.xml` / CMake / env-hook。
3. 用 `planar_move` + `p3d` + 空世界先把车刷进 Gazebo，`ros2 topic echo /cmd_vel` 能平移。
4. spawn 涵洞 SDF，确认碰撞。
5. 移植 Mid360 插件，确认 `/livox/lidar` 有点、涵洞墙壁上有回波。
6. 接上本仓库 `pointcloud_to_laserscan_node`，RViz 看 `/scan`。
7. 再开 `full_navigation.launch.py`。

### 3.9 方案 A 风险与折中

- 若 Mid360 重写周期太长：可临时用 `libgazebo_ros_ray_sensor.so` + `gpu_ray` 直接出 `/scan`，跳过点云投影。这会改变 LIO 输入，只适合先调 Nav2，不适合作为 Mid360/LIO 的最终方案。
- `planar_move` 忽略悬挂和轮地摩擦，TEB/MPC 的加速度限制可能要略收紧。
- Gazebo 11 与 ROS 2 时钟：务必全节点 `use_sim_time: true`。

---

## 4. 方案 B：Ignition Fortress + `ros_gz`（Humble 官方新仿真配对）

### 4.1 思路

Humble 官方文档把 **Fortress（Ignition Gazebo 6）** 配 `ros-humble-ros-gz`。launch / URDF / bridge 的**结构**可以继续像现在的 Jazzy 写法，但：

- CMake 从 `gz-sim8` 降到 `ignition-gazebo6`
- C++ 命名空间从 `gz::sim` / `gz::math` 回到 `ignition::gazebo` / `ignition::math`（Fortress 仍是 ignition 前缀）
- `parameter_bridge` 的消息类型要从 `gz.msgs.*` 核对成 Fortress 实际接受的名字（常见为 `ignition.msgs.*`，装好包后用 `--help` / 官方 demo 确认）
- 本机目前没装这套，需要 sudo

**优点**

- 与现有 `gazebo_sim.launch.py`、bridge 列表、gz system plugin 布局更像，launch 改动小于方案 A。
- Mid360 仍是 ECS `System` 插件，射线求交那套几何代码更接近现在，主要是命名空间和 CMake。
- 以后若有人在 Humble 上用 Fortress 世界，模型格式更接近 Harmonic 一侧（仍建议 SDF 降到 1.8/1.9 做兼容）。

**缺点**

- 本机要新装 Ignition Gazebo 6 全家桶，和已经在跑的 Classic 11 并存，容易 `gz`/`ign` 命令、插件路径打架。
- 仍然 **不是** Harmonic：不能直接 `find_package(gz-sim8)`。
- Bridge 类型字符串、world 名、resource path（`IGN_GAZEBO_RESOURCE_PATH` vs `GZ_SIM_RESOURCE_PATH`）都要按 Fortress 文档逐项核对。
- 麦轮 `VelocityControl`、差速 `DiffDrive` 在 Fortress 的 XML 标签可能与 Harmonic 略有差别（速度限制字段名等），要对照 Fortress 源码或 demo。

### 4.2 安装

```bash
sudo apt update
sudo apt install ros-humble-ros-gz
```

本机 apt 显示 `ros-humble-ros-gz-sim` 依赖 `libignition-gazebo6-dev`、`libignition-msgs8`、`libignition-transport11`。装完检查：

```bash
ign gazebo --version    # 应看到 Fortress / version 6
source /opt/ros/humble/setup.bash
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
```

不要用 Harmonic 的 `gz sim` 文档直接套命令。Fortress 入口经常是 `ign gazebo`。`ros_gz_sim` 的 launch 仍可能叫 `gz_sim.launch.py`，以包内文件为准。

### 4.3 依赖改动（`my_nav2_robot`）

`package.xml` 删除 vendor 包：

```xml
<!-- 删除 -->
<depend>gz_sim_vendor</depend>
<depend>gz_plugin_vendor</depend>
```

Humble 改为发行版 ros-gz（包名仍是 `ros_gz_*`）：

```xml
<depend>ros_gz_sim</depend>
<depend>ros_gz_bridge</depend>
<!-- 若 CMake 直接 find ignition： -->
<build_depend>ignition-gazebo6</build_depend>
```

`CMakeLists.txt` 示例：

```cmake
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)   # Fortress 插件注册，版本以 CMake 提示为准
find_package(ignition-math6 REQUIRED)

target_link_libraries(my_nav2_robot_mid360_gz_plugin
  ignition-gazebo6::core
  ignition-plugin1::register
)
```

**不要**再写

```cmake
HINTS /opt/ros/$ENV{ROS_DISTRO}/opt/gz_sim_vendor/lib/cmake
```

那是 Jazzy vendor 布局，Humble 没有。

env-hook：Fortress 常用 `IGN_GAZEBO_SYSTEM_PLUGIN_PATH`（部分 ros_gz 版本也读 `GZ_SIM_SYSTEM_PLUGIN_PATH`）。实施时在装好的 Fortress 上打印插件加载日志决定。`full_navigation.launch.py` 的 `GZ_SIM_RESOURCE_PATH` 同样改为 `IGN_GAZEBO_RESOURCE_PATH`，或两个都 set。

### 4.4 launch

可保留「Include `ros_gz_sim` launch + create spawn + parameter_bridge」结构。

必须改：

1. **world**：不要用仓库里不存在的 `sensors.sdf`。改成 Fortress 自带空世界，例如：

```python
'gz_args': '-r empty.sdf -v 3'
```

具体文件名以 `ros_gz_sim` / `ignition-gazebo6` 的 share 为准。涵洞仍用 `create -file culvert.sdf` spawn。

2. **create 可执行文件**：Humble `ros_gz_sim` 一般仍是 `executable='create'`，参数 `-world`、`-file`、`-topic robot_description` 与现在类似。装好后 `ros2 pkg executables ros_gz_sim` 确认。

3. **bridge 类型字符串**：当前 Jazzy 写法示例：

```
/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry
/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V
```

Fortress 上先跑官方 demo（`ros_gz_sim_demos`），把能通的 `@` / `[` / `]` 方向和 `ignition.msgs.*` vs `gz.msgs.*` 抄过来。方向含义保持：

- `]` ROS → Gazebo（如 `/cmd_vel`）
- `[` Gazebo → ROS（如 `/clock`、`/ground_truth`）
- `@` 双向

4. `/scan` 仍然 **不要** 从 Gazebo 2D 雷达桥，继续用 Mid360 点云 + 本仓库 `pointcloud_to_laserscan_node`。

5. 已注释的相机 / depth 桥可以继续注释。

### 4.5 URDF 插件对照（方案 B）

文件名和 `name` 从 Harmonic 的 `gz::sim::systems::*` 改到 Fortress 的 `ignition::gazebo::systems::*`。Fortress 插件 so 名常见为 `ignition-gazebo-*-system`，以 `/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/` 目录为准。

示意（实施时按目录里的真实 filename 填写）：

```xml
<plugin filename="ignition-gazebo-velocity-control-system"
        name="ignition::gazebo::systems::VelocityControl">
  <topic>/cmd_vel</topic>
</plugin>

<plugin filename="ignition-gazebo-diff-drive-system"
        name="ignition::gazebo::systems::DiffDrive">
  <!-- 关节、轮距、轮半径、/cmd_vel、/wheel_odom、/wheel_tf 保持现有语义 -->
</plugin>

<plugin filename="ignition-gazebo-joint-state-publisher-system"
        name="ignition::gazebo::systems::JointStatePublisher">
  <topic>/joint_states</topic>
</plugin>

<plugin filename="ignition-gazebo-odometry-publisher-system"
        name="ignition::gazebo::systems::OdometryPublisher">
  <odom_topic>/ground_truth</odom_topic>
  <odom_frame>odom</odom_frame>
  <robot_base_frame>base_footprint</robot_base_frame>
  <tf_topic>/tf</tf_topic>
  <dimensions>3</dimensions>
</plugin>
```

标签是否仍叫 `max_linear_velocity` 还是 Harmonic 的另一套名字，以 Fortress 6 的插件 README / 源码为准。差速「同侧前后两个 left_joint」若 Fortress 不支持多 joint，则与方案 A 一样只绑前轮。

### 4.6 Mid360 插件移植（方案 B）

保留 `System` + `Configure` + `PostUpdate` 结构。头文件替换对照：

| Harmonic (现在) | Fortress |
| --- | --- |
| `#include <gz/sim/System.hh>` | `#include <ignition/gazebo/System.hh>` |
| `#include <gz/sim/Model.hh>` | `#include <ignition/gazebo/Model.hh>` |
| `#include <gz/sim/EntityComponentManager.hh>` | `#include <ignition/gazebo/EntityComponentManager.hh>` |
| `#include <gz/math/Pose3.hh>` | `#include <ignition/math/Pose3.hh>` |
| `gz::sim::kNullEntity` | `ignition::gazebo::kNullEntity` |
| `GZ_ADD_PLUGIN(...)` | `IGNITION_ADD_PLUGIN(...)`（头文件以 plugin1 文档为准） |

求交函数里的 `gz::math::Vector3d` 全部换成 `ignition::math::Vector3d`。ECM 上取 `Pose` / `Geometry` / `Collision` 组件的 **类型名** 在 Fortress 里带 `ignition::gazebo::components::` 前缀；若某个 Harmonic 组件在第 6 版不存在，再改用相邻组件或退回 world 射线。

ROS 发布部分（rclcpp、`PointCloud2`、CSV）与方案 A 一样可以原样保留。

### 4.7 SDF

- `sdf version="1.10"` 对 Fortress 偏新，建议降到 **1.8** 或 **1.9** 再在 `ign gazebo` 里单独打开 `culvert.sdf` 看报错。
- 资源路径：`IGN_GAZEBO_RESOURCE_PATH=$COLCON_PREFIX_PATH/my_nav2_robot/share:...`

### 4.8 方案 B 话题契约

目标与现在 Jazzy 相同：bridge 列表里的 `/cmd_vel`、`/joint_states`、`/ground_truth`、`/clock`，可选 `/tf`。`/livox/lidar` 由自定义插件 **直接发 ROS**（和现在一样不走 bridge，因为现插件内部已经 `rclcpp::Publisher`）。这一点方案 A/B 相同。

注意：Fortress 插件里直接用 rclcpp 发点云，仍要保证 `use_sim_time` 与 `/clock` 桥已起来，否则 LIO 时间戳会飘。

### 4.9 方案 B 实施顺序

1. 第 2 节共用改动先编过。
2. `sudo apt install ros-humble-ros-gz`，跑通 `ros_gz_sim_demos` 里最简单的例子。
3. 记下 demo 里的 bridge 类型字符串、launch 文件名、空世界名字。
4. 改 CMake/package.xml，先不加载 Mid360，用 Fortress DiffDrive/VelocityControl 把车 spawn 进空世界。
5. 打通 `/cmd_vel` 与 `/clock`。
6. 按命名空间移植 Mid360，确认 `/livox/lidar`。
7. spawn 涵洞，再接 Nav2。

### 4.10 方案 B 风险

- `gz` CLI 与 `ign` CLI 同时存在时，脚本里写错命令会启动错误的仿真器。
- Humble 的 `ros_gz` 补丁版本之间 bridge 语法有过改动，**以本机 demo 为准**，不要只抄 Jazzy 的 `gz.msgs`。
- 若 apt 的 Fortress 与本机 Classic 11 抢 OpenGL / 模型路径，出现「世界是空的」时先 `echo $IGN_GAZEBO_RESOURCE_PATH`、`$GAZEBO_MODEL_PATH`。

---

## 5. 方案 A vs 方案 B（怎么选）

| 维度 | 方案 A Classic 11 | 方案 B Fortress |
| --- | --- | --- |
| 额外 apt | 几乎无 | 必须 `ros-humble-ros-gz`（Ignition 6） |
| 与本机现状 | 已装齐 | 尚未安装 |
| launch 改动量 | 大（整段 gz 启动/桥接重写） | 中（结构可留，world/类型字符串要改） |
| URDF 驱动 | 换成 `gazebo_ros_*` | 保留 system plugin 思路，改 ignition 前缀 |
| Mid360 | 改 ModelPlugin，求交后端换物理引擎 | 保留 System 插件，主要是命名空间/CMake |
| 与 Jazzy `main` 的 diff | 仿真层会长期分叉 | 仿真层 diff 较小，仍无法直接合并 Harmonic |
| 和 LIO 的接近程度 | 点云格式可保持；运动学更「滑」 | 运动学更接近现在的 VelocityControl |
| 推荐场景 | 尽快在这台 22.04 机器上跑通导航 | 希望少改 launch/URDF 结构，并接受装 Fortress |

**文档建议（非强制）：** 这台已经装好 Classic 11 的机器用方案 A 更快落地；若后续有 Humble 机专门跑 Ignition、或希望尽量贴近 `main` 的 gz 插件代码，再做方案 B。

两套方案 **不要混在同一套 launch 里用 if 切换**，除非单独做成 `sim_backend:=classic|fortress` 并准备两份 URDF。第一轮只落地其中一个。

---

## 6. 推荐落地顺序（选定 A 或 B 之后）

```text
第 0 步  conda deactivate；确认 python3 为 3.10
         sudo apt install ros-humble-osqp-vendor
         （仅 B）sudo apt install ros-humble-ros-gz

第 1 步  第 2 节：规划器签名、异常头、BT v3、yaml、README
         colcon build --packages-skip nav2_visual_processing my_nav2_robot
         （先不编仿真包也可以先编规划器）

第 2 步  按选定方案改 my_nav2_robot 的 CMake / URDF / launch / Mid360
         只跑 gazebo_sim.launch.py

第 3 步  full_navigation.launch.py
         chassis:=mecanum / diff，slam:=False / True 各一次

第 4 步  NavigateToPose 走通 A* → SmoothPath → ManageTrajectory → FollowPath
```

### 6.1 仿真层验收清单

- Gazebo 窗口里有车、有涵洞或 map1 障碍。
- `ros2 topic hz /clock`、`/livox/lidar`、`/scan`、`/ground_truth` 都有数据。
- `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...` 车能动。
- `use_gazebo_odom_tf:=false` 时 TF 树 **没有** 第二份 `odom → base_footprint`。

### 6.2 Nav2 验收清单

- `ros2 lifecycle list` 里 Humble 那 7 个 navigation 节点为 active。
- BT 能加载 `test_nav.xml`（无 `BTCPP_format="4"` 报错）。
- 自定义插件能 `ros2 param get` 到 `planner_server` / `controller_server` / `smoother_server` 里的 plugin 名。
- 发目标后 `/cmd_vel` 有输出，代价地图能看到 `/scan` 障碍。

---

## 7. 明确不做（第一轮）

- 不改 `main`，不把 Humble 补丁回灌 Jazzy。
- 不移植 `docking_server`。
- 不把 `collision_monitor` 塞进 Humble bringup（除非单独写 launch）。
- 不安 `small_gicp`、不编译 `nav2_visual_processing`。
- 不在本机用 conda Python 3.13 跑 `ros2 launch`。

---

## 8. 关键文件索引

共用：

- `my_nav2_planner/`、`my_rrtstar_planner/`、`my_hybrid_astar_planner/`（`createPlan` + 异常头）
- `my_nav2_controller/`、`my_mpc_controller/`、`my_teb_controller/`（异常头）
- `my_nav2_bt_nodes/`（BT v3）
- `my_nav2_robot/behavior_trees/test_nav.xml`、`test_nav_diff.xml`
- `my_nav2_robot/config/nav2_params_*.yaml`（四份）
- `my_nav2_robot/launch/full_navigation.launch.py`
- `README.md`

方案 A 额外：

- `my_nav2_robot/package.xml`、`CMakeLists.txt`、`env-hooks/my_nav2_robot.dsv.in`
- `my_nav2_robot/launch/gazebo_sim.launch.py`（重写）
- `my_nav2_robot/urdf/robot.urdf.xacro`、`robot_diff.urdf.xacro`
- `my_nav2_robot/include/my_nav2_robot/mid360_gz_plugin.hpp`
- `my_nav2_robot/src/mid360_gz_plugin.cpp`
- `my_nav2_robot/models/culvert.sdf`、`models/map1_obstacles/`

方案 B 额外：

- 同上 URDF / Mid360 / CMake / launch，但是 **改 ignition 前缀和 bridge 字符串**，而不是换成 `gazebo_ros_*`
- 本机 apt 安装 `ros-humble-ros-gz`

`pointcloud_to_laserscan.launch.py`、`simulated_imu_publisher`、`mid360.csv` 在 A/B 下都保持现有话题约定。
