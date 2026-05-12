# my_teb_controller 实现规划文档

## 1. 概述

### 1.1 目标
在现有 `my_teb_controller` 骨架包基础上，补全构建系统依赖、插件注册、代码框架（数据结构 + 类成员 + 方法声明），核心算法方法体留空由用户填写。

### 1.2 TEB 算法简介
TEB（Timed Elastic Band）是一种局部轨迹优化方法。将机器人轨迹表示为一系列带时间间隔的位姿点（PoseSE2 + Δt），通过 g2o 图优化框架最小化以下代价之和：
- **障碍物代价**：轨迹点与障碍物的距离惩罚
- **路径跟随代价**：轨迹与全局路径的偏差
- **速度/加速度平滑代价**：相邻时间间隔的一致性
- **时间最优代价**：鼓励较短的总行驶时间
- **可行性代价**：满足运动学约束（如最小转弯半径）

### 1.3 项目内参考包
所有风格严格对齐以下两个已验证的控制器插件：
- `my_mpc_controller`（MPC 控制器，OSQP 求解）
- `my_nav2_controller`（DWA 控制器）

---

## 2. 文件清单

### 2.1 需要修改的文件（已存在）

| 文件 | 修改内容 |
|------|----------|
| `package.xml` | 补全所有 `<depend>` 声明，添加 `<nav2_core plugin>` 导出 |
| `CMakeLists.txt` | 添加 `find_package`、`add_library`、`ament_target_dependencies`、安装规则、导出规则 |
| `include/my_teb_controller/teb_controller.hpp` | 补全数据结构、类成员变量、protected/private 方法声明 |
| `src/teb_controller.cpp` | 补全 `#include`、方法骨架（空方法体）、`PLUGINLIB_EXPORT_CLASS` |

### 2.2 需要新建的文件

| 文件 | 用途 |
|------|------|
| `plugins.xml` | pluginlib 插件描述文件 |

---

## 3. 构建系统配置

### 3.1 `package.xml` 完整依赖清单

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
  schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_teb_controller</name>
  <version>0.0.0</version>
  <description>TEB局部规划器控制器插件</description>
  <maintainer email="meis38@126.com">goose</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Nav2 核心 -->
  <depend>nav2_core</depend>
  <depend>nav2_util</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>pluginlib</depend>

  <!-- ROS2 基础 -->
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>visualization_msgs</depend>

  <!-- TF -->
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <!-- 数学库 -->
  <depend>Eigen3</depend>
  <depend>angles</depend>

  <!-- 优化求解器 -->
  <depend>osqp</depend>
  <depend>osqp_vendor</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/plugins.xml" />
  </export>
</package>
```

**依赖说明：**
- `nav2_core`：Controller 基类接口
- `nav2_util`：`declare_parameter_if_not_declared` 等工具
- `nav2_costmap_2d`：代价地图，用于障碍物查询
- `pluginlib`：插件注册与加载
- `rclcpp` / `rclcpp_lifecycle`：ROS2 节点与生命周期
- `geometry_msgs`：TwistStamped, PoseStamped 等消息
- `nav_msgs`：Path 消息
- `visualization_msgs`：MarkerArray 可视化（TEB 轨迹在 RViz 中显示）
- `tf2` / `tf2_ros` / `tf2_geometry_msgs`：坐标变换
- `Eigen3`：矩阵运算（代价矩阵构建）
- `angles`：角度归一化工具
- `osqp` / `osqp_vendor`：二次规划求解器（与 MPC/BSpline 一致）

### 3.2 `CMakeLists.txt` 完整内容

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_teb_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)
find_package(osqp REQUIRED)
find_package(osqp_vendor REQUIRED)

# 添加头文件目录
include_directories(include)

# 编译动态库 (Shared Library)
add_library(${PROJECT_NAME}_lib SHARED
  src/teb_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  nav2_core
  nav2_util
  nav2_costmap_2d
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  angles
  osqp
)

target_link_libraries(${PROJECT_NAME}_lib
  osqp::osqp
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# 导出依赖以供其他包使用
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(nav2_core nav2_util pluginlib rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

**与 MPC 包的差异：**
- 新增 `visualization_msgs`（TEB 轨迹 Marker 可视化）
- 新增 `tf2_geometry_msgs`（位姿 TF 变换）
- 新增 `angles`（角度归一化）

### 3.3 `plugins.xml`

```xml
<library path="my_teb_controller_lib">
  <class name="my_teb_controller/MyTebController"
         type="my_teb_controller::MyTebController"
         base_class_type="nav2_core::Controller">
    <description>
      这是一个TEB局部规划器控制器插件
    </description>
  </class>
</library>
```

**命名规范对齐 MPC 包：**
- `library path` = `${PROJECT_NAME}_lib`（与 CMakeLists.txt 中 `add_library` 的目标名一致）
- `class name` = `包名/类名`
- `type` = `命名空间::类名`
- `base_class_type` = `nav2_core::Controller`

---

## 4. 头文件设计

### 4.1 文件路径
`include/my_teb_controller/teb_controller.hpp`

### 4.2 头文件保护宏
```cpp
#ifndef MY_TEB_CONTROLLER__TEB_CONTROLLER
#define MY_TEB_CONTROLLER__TEB_CONTROLLER
// ...
#endif // MY_TEB_CONTROLLER__TEB_CONTROLLER
```
对齐 MPC：`包名大写__文件名大写`，无尾部下划线。

### 4.3 Include 顺序
遵循 AGENTS.md 规范：
1. 对应头文件（.cpp 中）
2. C++ 标准库
3. 第三方库（Eigen）
4. ROS2/Nav2 头文件
5. 包内头文件

```cpp
// C++ 标准库
#include <memory>
#include <string>
#include <vector>
#include <cmath>

// 第三方库
#include <Eigen/Dense>

// ROS2 / Nav2
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
```

### 4.4 数据结构设计

TEB 算法需要的核心数据结构，放在 `my_teb_controller` 命名空间下、类定义之前：

#### 4.4.1 `PoseSE2` — 二维位姿 (x, y, theta)
```cpp
struct PoseSE2 {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    PoseSE2() = default;
    PoseSE2(double x, double y, double theta);

    Eigen::Vector2d position() const;
    void setTheta(double theta);
    double thetaMod(double theta) const;
};
```

#### 4.4.2 `TimedPose` — 带时间间隔的位姿
```cpp
struct TimedPose {
    PoseSE2 pose;
    double dt = 0.0;  // 到下一个位姿的时间间隔

    TimedPose() = default;
    TimedPose(const PoseSE2 &pose, double dt);
};
```

#### 4.4.3 `TebConfig` — TEB 参数配置
```cpp
struct TebConfig {
    // 轨迹参数
    double teb_autosize = true;        // 自动调整轨迹点数
    double dt_ref = 0.3;               // 参考时间间隔 (s)
    double dt_hysteresis = 0.1;        // 时间间隔滞后带 (s)
    int min_samples = 3;               // 最少轨迹点数
    int max_samples = 500;             // 最多轨迹点数

    // 障碍物参数
    double min_obstacle_dist = 0.3;    // 最小障碍物距离 (m)
    double inflation_dist = 0.5;       // 障碍物膨胀距离 (m)
    double costmap_weight = 100.0;     // costmap 代价权重

    // 优化参数
    int no_iterations = 5;             // 每次规划的优化迭代次数
    double penalty_epsilon = 0.1;      // 违约惩罚松弛量
    double weight_obstacle = 50.0;     // 障碍物代价权重
    double weight_optimaltime = 1.0;   // 时间最优权重
    double weight_shortest_path = 0.0; // 最短路径权重
    double weight_smoothness = 100.0;  // 平滑性权重
    double weight_kinematics = 500.0;  // 运动学约束权重

    // 机器人参数
    double max_vel_x = 0.5;            // 最大线速度 (m/s)
    double max_vel_x_backwards = 0.2;  // 最大倒车速度 (m/s)
    double max_vel_theta = 1.0;        // 最大角速度 (rad/s)
    double acc_lim_x = 2.0;            // 最大线加速度 (m/s^2)
    double acc_lim_theta = 1.5;        // 最大角加速度 (rad/s^2)
    double min_turning_radius = 0.0;   // 最小转弯半径 (m), 0=差速驱动

    // 底盘类型
    bool is_holonomic = false;          // 是否全向底盘（麦轮=true）
};
```

### 4.5 类定义

```cpp
namespace my_teb_controller {

class MyTebController : public nav2_core::Controller {

public:

    MyTebController() = default;
    ~MyTebController() override = default;

    // ========== Nav2 插件接口（与 MPC/DWA 完全一致） ==========

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void activate() override;
    void deactivate() override;
    void cleanup() override;

    void setPlan(const nav_msgs::msg::Path &path) override;

    void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &v,
        nav2_core::GoalChecker */*goal_checker*/) override;

protected:

    // ========== TEB 核心算法方法（用户填写方法体） ==========

    /**
     * @brief 将全局路径转换为初始 TEB 轨迹
     * @param global_path 全局路径
     * @param start_pose 当前机器人位姿
     */
    void initializeTrajectory(
        const nav_msgs::msg::Path &global_path,
        const PoseSE2 &start_pose);

    /**
     * @brief 执行 TEB 图优化迭代
     */
    void optimizeTEB();

    /**
     * @brief 从优化后的轨迹提取速度指令
     * @param robot_pose 当前机器人位姿
     * @param robot_vel 当前机器人速度
     * @return 速度指令
     */
    geometry_msgs::msg::TwistStamped extractVelocity(
        const PoseSE2 &robot_pose,
        const geometry_msgs::msg::Twist &robot_vel);

    /**
     * @brief 检查轨迹可行性（碰撞 + 运动学）
     * @return true 如果轨迹可行
     */
    bool checkTrajectoryFeasibility();

    /**
     * @brief 自动调整轨迹点数（添加/删除点）
     */
    void autoResizeTrajectory();

    /**
     * @brief 查询 costmap 获取某点的障碍物距离
     * @param x 世界坐标 x
     * @param y 世界坐标 y
     * @return 到最近障碍物的距离 (m)，如果在地图外返回 -1
     */
    double getObstacleDistance(double x, double y);

    /**
     * @brief 发布 TEB 轨迹可视化 Marker
     */
    void publishVisualization();

    // ========== 成员变量 ==========

    // ROS2 基础（对齐 MPC 包命名风格）
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    rclcpp::Logger logger_{rclcpp::get_logger("MyTebController")};
    rclcpp::Clock::SharedPtr clock_;
    std::string plugin_name_;

    // 全局路径
    nav_msgs::msg::Path global_plan_;

    // TEB 轨迹
    std::vector<TimedPose> teb_trajectory_;

    // TEB 配置
    TebConfig config_;

    // 上一次速度指令（用于加速度约束）
    double last_v_x_ = 0.0;
    double last_v_y_ = 0.0;
    double last_omega_ = 0.0;

    // 调试可视化
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr teb_marker_pub_;

    // TF 容差
    rclcpp::Duration transform_tolerance_{0, 0};
};

} // my_teb_controller
```

### 4.6 设计决策说明

| 决策 | 理由 |
|------|------|
| `PoseSE2` / `TimedPose` 放在类外、命名空间内 | 与 MPC 的 `PathPoint` / `MPCmatrices` 风格一致 |
| `TebConfig` 用 struct 而非 ROS 参数直接映射 | TEB 参数较多，统一管理更清晰，`configure()` 中一次性从参数服务器读取 |
| `protected` 而非 `private` | 方便未来子类继承扩展，与 MPC 一致 |
| 轨迹用 `std::vector<TimedPose>` | 简单直观，不引入 g2o 依赖（用户可自行替换） |
| `is_holonomic` 参数 | 本项目机器人是麦轮（全向），需要区分全向/差速运动学约束 |
| 可视化用 `MarkerArray` | TEB 轨迹需要显示位姿点 + 时间间隔箭头，比 Path 更丰富 |

---

## 5. 源文件设计

### 5.1 文件路径
`src/teb_controller.cpp`

### 5.2 Include
```cpp
#include "my_teb_controller/teb_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
```
对齐 MPC：第一行对应头文件，后续依次是 nav2_core、pluginlib、nav2_util。

### 5.3 方法实现骨架

#### 5.3.1 `PoseSE2` 构造与工具方法
```cpp
namespace my_teb_controller {

PoseSE2::PoseSE2(double x, double y, double theta)
    : x(x), y(y), theta(theta) {}

Eigen::Vector2d PoseSE2::position() const {
    return Eigen::Vector2d(x, y);
}

void PoseSE2::setTheta(double theta) {
    this->theta = theta;
}

double PoseSE2::thetaMod(double theta) const {
    // 归一化到 [-pi, pi]
    // 用户填写
    return 0.0;
}
```

#### 5.3.2 `TimedPose` 构造
```cpp
TimedPose::TimedPose(const PoseSE2 &pose, double dt)
    : pose(pose), dt(dt) {}
```

#### 5.3.3 `configure()`
对齐 MPC 风格：先锁节点、存基础指针，再批量声明+获取参数，最后创建 Publisher。

```cpp
void MyTebController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {

    node_ = parent;
    auto node = node_.lock();
    tf_ = tf;
    plugin_name_ = name;
    costmap_ros_ = costmap_ros;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // 参数声明与获取（对齐 MPC 的 declare + get 两步模式）
    // 轨迹参数
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".dt_ref", rclcpp::ParameterValue(0.3));
    node->get_parameter(plugin_name_ + ".dt_ref", config_.dt_ref);
    // ... 其余参数同理，每个参数对应一对 declare + get ...

    // TF 容差
    double transform_tolerance;
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

    // 调试可视化 Publisher
    teb_marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        plugin_name_ + "/teb_markers", 10);

    RCLCPP_INFO(logger_, "自定义TEB控制器配置完成");
}
```

**参数清单（每个参数的默认值对应 `TebConfig` 中的初始值）：**

| 参数名 | 默认值 | 对应 config_ 成员 |
|--------|--------|-------------------|
| `dt_ref` | 0.3 | dt_ref |
| `dt_hysteresis` | 0.1 | dt_hysteresis |
| `min_samples` | 3 | min_samples |
| `max_samples` | 500 | max_samples |
| `min_obstacle_dist` | 0.3 | min_obstacle_dist |
| `inflation_dist` | 0.5 | inflation_dist |
| `no_iterations` | 5 | no_iterations |
| `weight_obstacle` | 50.0 | weight_obstacle |
| `weight_optimaltime` | 1.0 | weight_optimaltime |
| `weight_smoothness` | 100.0 | weight_smoothness |
| `weight_kinematics` | 500.0 | weight_kinematics |
| `max_vel_x` | 0.5 | max_vel_x |
| `max_vel_x_backwards` | 0.2 | max_vel_x_backwards |
| `max_vel_theta` | 1.0 | max_vel_theta |
| `acc_lim_x` | 2.0 | acc_lim_x |
| `acc_lim_theta` | 1.5 | acc_lim_theta |
| `min_turning_radius` | 0.0 | min_turning_radius |
| `is_holonomic` | true | is_holonomic |
| `transform_tolerance` | 0.1 | transform_tolerance_ |

#### 5.3.4 `setPlan()`
```cpp
void MyTebController::setPlan(const nav_msgs::msg::Path &path) {
    global_plan_ = path;
}
```

#### 5.3.5 `computeVelocityCommands()`
对齐 MPC 的整体流程框架：
```cpp
geometry_msgs::msg::TwistStamped MyTebController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker */*goal_checker*/) {

    auto node = node_.lock();

    // 1. 获取当前机器人位姿
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double theta = tf2::getYaw(pose.pose.orientation);
    PoseSE2 robot_pose(x, y, theta);

    // 2. 初始化轨迹（如果是新路径或首次调用）
    //    调用 initializeTrajectory()

    // 3. 执行 TEB 优化迭代
    //    调用 optimizeTEB()

    // 4. 检查轨迹可行性
    //    调用 checkTrajectoryFeasibility()

    // 5. 提取速度指令
    //    调用 extractVelocity()

    // 6. 发布可视化
    //    调用 publishVisualization()

    // 7. 封装 TwistStamped（对齐 MPC 的封装方式）
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = "base_link";
    // cmd_vel.twist.linear.x = ...
    // cmd_vel.twist.linear.y = ...  （全向底盘）
    // cmd_vel.twist.angular.z = ...

    return cmd_vel;
}
```

#### 5.3.6 TEB 算法方法（空方法体）
```cpp
void MyTebController::initializeTrajectory(
    const nav_msgs::msg::Path &global_path,
    const PoseSE2 &start_pose) {
    // 用户填写：从全局路径采样生成初始 teb_trajectory_
}

void MyTebController::optimizeTEB() {
    // 用户填写：构建优化问题并求解
}

geometry_msgs::msg::TwistStamped MyTebController::extractVelocity(
    const PoseSE2 &robot_pose,
    const geometry_msgs::msg::Twist &robot_vel) {
    // 用户填写：从 teb_trajectory_ 的前两个点计算速度指令
    geometry_msgs::msg::TwistStamped cmd_vel;
    return cmd_vel;
}

bool MyTebController::checkTrajectoryFeasibility() {
    // 用户填写：检查碰撞和运动学约束
    return true;
}

void MyTebController::autoResizeTrajectory() {
    // 用户填写：根据距离/时间自动增删轨迹点
}

double MyTebController::getObstacleDistance(double x, double y) {
    // 用户填写：查询 costmap
    return -1.0;
}

void MyTebController::publishVisualization() {
    // 用户填写：发布 MarkerArray 到 teb_marker_pub_
}
```

#### 5.3.7 生命周期方法
对齐 MPC 的极简风格：
```cpp
void MyTebController::activate() { RCLCPP_INFO(logger_, "插件已激活"); }
void MyTebController::deactivate() { RCLCPP_INFO(logger_, "插件已停用"); }
void MyTebController::cleanup() { RCLCPP_INFO(logger_, "插件已清理"); }
```

#### 5.3.8 `setSpeedLimit()`
对齐 MPC 的 TODO 风格：
```cpp
void MyTebController::setSpeedLimit(const double &speed_limit, const bool &percentage) {
    (void)speed_limit;
    (void)percentage;
    RCLCPP_INFO(logger_, "收到限速指令，当前插件尚未实现具体的限速逻辑");
}
```

#### 5.3.9 插件注册（文件最后一行）
```cpp
} // my_teb_controller

// 注册算法插件
PLUGINLIB_EXPORT_CLASS(my_teb_controller::MyTebController, nav2_core::Controller)
```

---

## 6. 代码风格规范

### 6.1 格式
- **缩进**：4 空格，无 tab
- **花括号**：K&R 风格（开括号同行）
- **行宽**：120 字符以内
- **空行**：方法之间空一行，逻辑块之间空一行

### 6.2 命名
| 类型 | 规范 | 示例 |
|------|------|------|
| 类名 | PascalCase | `MyTebController` |
| 方法名 | camelCase | `computeVelocityCommands` |
| 成员变量 | snake_case 带尾部下划线 | `plugin_name_`, `teb_trajectory_` |
| 局部变量 | snake_case | `robot_pose`, `min_dist` |
| 常量 | UPPER_SNAKE_CASE | `MIN_SAMPLES` |
| 文件名 | snake_case | `teb_controller.cpp` |
| 命名空间 | 小写+下划线 | `my_teb_controller` |
| 结构体 | PascalCase | `PoseSE2`, `TebConfig` |

### 6.3 注释
- 通用注释用英文
- 项目特定逻辑可用中文（与 MPC/DWA 一致）
- Doxygen `@brief` / `@param` / `@return` 用于 public/protected 方法声明

### 6.4 参数声明模式
严格两步，对齐 MPC：
```cpp
nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".param_name", rclcpp::ParameterValue(default_value));
node->get_parameter(plugin_name_ + ".param_name", member_variable);
```

### 6.5 日志模式
```cpp
RCLCPP_INFO(logger_, "自定义TEB控制器配置完成");
RCLCPP_WARN(logger_, "xxx");
RCLCPP_ERROR(logger_, "xxx");
```

---

## 7. Nav2 配置集成

### 7.1 在 `nav2_params_nav_diy.yaml` 中添加 TEB 配置

在 `controller_server` 的 `ros__parameters` 下添加 TEB 控制器配置（作为 MPPI/MPC 的替代选项）：

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    # 切换到 TEB 控制器时使用：
    # controller_plugins: ["FollowPath"]
    # FollowPath:
    #   plugin: "my_teb_controller/MyTebController"
    #   dt_ref: 0.3
    #   dt_hysteresis: 0.1
    #   min_samples: 3
    #   max_samples: 500
    #   min_obstacle_dist: 0.3
    #   inflation_dist: 0.5
    #   no_iterations: 5
    #   weight_obstacle: 50.0
    #   weight_optimaltime: 1.0
    #   weight_smoothness: 100.0
    #   weight_kinematics: 500.0
    #   max_vel_x: 0.5
    #   max_vel_x_backwards: 0.2
    #   max_vel_theta: 1.0
    #   acc_lim_x: 2.0
    #   acc_lim_theta: 1.5
    #   min_turning_radius: 0.0
    #   is_holonomic: true
    #   transform_tolerance: 0.1
```

---

## 8. 构建与验证

### 8.1 构建命令
```bash
cd ~/nav2_test
colcon build --packages-select my_teb_controller
```

### 8.2 验证步骤
1. **编译通过**：`colcon build --packages-select my_teb_controller` 无错误
2. **插件加载**：启动 Nav2 后在日志中确认 `自定义TEB控制器配置完成`
3. **参数生效**：通过 `ros2 param list /controller_server` 确认参数注册
4. **可视化**：在 RViz2 中添加 MarkerArray 显示 `my_teb_controller/teb_markers` 话题

### 8.3 常见问题
- 若 `PLUGINLIB_EXPORT_CLASS` 报链接错误：检查 `plugins.xml` 的 `library path` 是否与 `add_library` 目标名一致
- 若参数未声明：检查 `declare_parameter_if_not_declared` 的 `plugin_name_` 前缀是否正确
- 若 costmap 查询返回 255：确认 `costmap_ros_->getCostmap()` 指针有效

---

## 9. 任务执行顺序

1. **Step 1**：新建 `plugins.xml`
2. **Step 2**：重写 `package.xml`（补全依赖）
3. **Step 3**：重写 `CMakeLists.txt`（补全构建逻辑）
4. **Step 4**：重写 `teb_controller.hpp`（补全数据结构 + 成员 + 方法声明）
5. **Step 5**：重写 `teb_controller.cpp`（补全方法骨架 + 插件注册）
6. **Step 6**：`colcon build --packages-select my_teb_controller` 验证编译通过
