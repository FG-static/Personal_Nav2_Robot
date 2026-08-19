# 基于$\text{Nav2}$的一个机器人仿真项目
## 环境要求
`humble` 分支在 **Ubuntu 22.04 + ROS 2 Humble** 上构建，仿真走 **方案 A：Gazebo Classic 11 + `gazebo_ros`**（见 [`HUMBLE_PORTING.md`](HUMBLE_PORTING.md) 第 3 节）。上游 `main` 仍是 Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic。推荐用 rosdep 补依赖。2D 扫描由本仓库 `my_nav2_robot/pointcloud_to_laserscan_node` 从 Mid360 点云投影生成，**不需要** `apt install ros-humble-pointcloud-to-laserscan`。

在工作区根目录也可以用 rosdep 一次补齐：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 运行须知
本项目的`my_nav2_robot`是小车的核心功能包，其余功能包基本上均为为nav2服务的各种导航器插件。配置小车本体属性的`.urdf`文件、配置小车搭载的各种插件的`.yaml`文件、启动文件均在里面，请在确保知道自己在干什么的情况下再进行内部的修改，否则可能会导致项目无法启动
本项目内置的bt的`.xml`文件和地图文件`.pgm`以及`.yaml`，可以自行替换，同时在launch文件也同步修改路径

## `/scan` 来源（点云水平切片）

实车没有 2D 雷达。仿真里也已去掉 Gazebo `gpu_lidar`，改为把 Mid360 的 `sensor_msgs/PointCloud2`（`/livox/lidar`）投影成 `sensor_msgs/LaserScan`（`/scan`），AMCL、SLAM Toolbox、costmap 的话题结构保持不变。

| 项目 | 说明 |
| --- | --- |
| 实现 | 本仓库 `my_nav2_robot` 的 `pointcloud_to_laserscan_node`（算法对齐官方 Humble `pointcloud_to_laserscan`，不依赖该 apt 包） |
| 节点名 | `pointcloud_to_laserscan` |
| 输入 | `/livox/lidar`（`livox_frame`） |
| 输出 | `/scan`（`laser_link`，Best Effort，与 AMCL / SLAM Toolbox / costmap 一致） |
| 参数 | `src/my_nav2_robot/config/pointcloud_to_laserscan.yaml` |
| 启动 | `gazebo_sim.launch.py` 会自动带上；也可单独 `ros2 launch my_nav2_robot pointcloud_to_laserscan.launch.py` |

`laser_link` 只是虚拟 2D 扫描坐标系，不是物理传感器。切片高度、量程需要按 Mid360 安装高度在实车上重标。

## 快速开始
使用`my_nav2_robot`下的`launch`下的`full_navigation.launch.py`launch文件进行项目启动，启动前应使用
```bash
. install/setup.bash
```
刷新系统环境，以下命令简单启动项目：
```bash
ros2 launch my_nav2_robot full_navigation.launch.py
```
无头仿真（不启 gzclient / 可用 `use_gazebo_odom_tf:=true` 先验证里程计 TF）：
```bash
ros2 launch my_nav2_robot gazebo_sim.launch.py gui:=false use_gazebo_odom_tf:=true scene:=culvert
```
同时，还可以使用`slam`参数在运行launch文件时设置是否打开slam模式
