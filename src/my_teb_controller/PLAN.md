# my_teb_controller 实现规划文档（麦克纳姆轮版）

## 1. 概述

### 1.1 目标
在现有 `my_teb_controller` 骨架包基础上，补全构建系统依赖、插件注册、代码框架（数据结构 + 类成员 + 方法声明），核心算法方法体留空由用户填写。**本规划面向麦克纳姆轮（全向）底盘，控制量为 (v_x, v_y, omega)。**

### 1.2 TEB 算法简介
TEB（Timed Elastic Band）是一种局部轨迹优化方法。将机器人轨迹表示为一系列带时间间隔的位姿点（PoseSE2 + Δt），通过图优化最小化以下代价之和：
- **障碍物代价**：轨迹点与障碍物的距离惩罚
- **路径跟随代价**：轨迹与全局路径的偏差
- **速度/加速度平滑代价**：相邻时间间隔的一致性
- **时间最优代价**：鼓励较短的总行驶时间
- **运动学代价**：满足麦克纳姆轮运动学约束

### 1.3 麦克纳姆轮 vs 差速轮 — TEB 关键差异

| 维度 | 差速轮 | 麦克纳姆轮（本项目） |
|------|--------|---------------------|
| 控制量 | (v, omega) 2维 | **(v_x, v_y, omega) 3维** |
| 横向运动 | 不支持 | **支持（v_y ≠ 0）** |
| 原地旋转 | 不支持（需转弯半径） | **支持（min_turning_radius = 0）** |
| 倒车速度限制 | 需要 max_vel_x_backwards | **不需要，任意方向等价** |
| 世界坐标速度变换 | x' = v*cos(θ), y' = v*sin(θ) | **x' = v_x*cos(θ) - v_y*sin(θ), y' = v_x*sin(θ) + v_y*cos(θ)** |
| 优化变量维度 | 每个时间步 2 个控制变量 | **每个时间步 3 个控制变量** |

### 1.4 麦轮运动学模型（与 MPC 包一致）

```
世界坐标系下的离散运动方程：
    x_{k+1}     x_k     (v_x * cos(θ) - v_y * sin(θ)) * Δt
    y_{k+1}  =  y_k  +  (v_x * sin(θ) + v_y * cos(θ)) * Δt
    θ_{k+1}     θ_k     omega * Δt

控制输入向量 u = [v_x, v_y, omega]^T
```

参考 MPC 包的 B 矩阵（`mpc_controller.cpp:298`）：
```
B = [cos(θ)*Δt  -sin(θ)*Δt  0    ]
    [sin(θ)*Δt   cos(θ)*Δt  0    ]
    [0            0           Δt  ]
```

### 1.5 项目内参考包
所有风格严格对齐以下两个已验证的控制器插件：
- `my_mpc_controller`（MPC 控制器，OSQP 求解）— **麦轮运动学参考**
- `my_nav2_controller`（DWA 控制器）

---

## 2. 文件清单

### 2.1 需要修改的文件（已存在）

| 文件 | 修改内容 |
|------|----------|
| `package.xml` | 补全所有 `<depend>` 声明，添加 `<nav2_core plugin>` 导出 |
| `CMakeLists.txt` | 添加 `find_package`、`add_library`、`ament_target_dependencies`、安装规则、导出规则 |
| `include/my_teb_controller/teb_controller.hpp` | 补全数据结构、类成员变量、protected 方法声明 |
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
  <description>TEB局部规划器控制器插件（麦克纳姆轮）</description>
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

### 3.3 `plugins.xml`

```xml
<library path="my_teb_controller_lib">
  <class name="my_teb_controller/MyTebController"
         type="my_teb_controller::MyTebController"
         base_class_type="nav2_core::Controller">
    <description>
      这是一个TEB局部规划器控制器插件（麦克纳姆轮）
    </description>
  </class>
</library>
```

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

### 4.3 Include 顺序
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

#### 4.4.3 `MecanumVelocity` — 麦轮三自由度速度
```cpp
struct MecanumVelocity {
    double v_x = 0.0;    // 车体系 x 方向速度 (m/s)
    double v_y = 0.0;    // 车体系 y 方向速度 (m/s)
    double omega = 0.0;  // 车体系角速度 (rad/s)

    MecanumVelocity() = default;
    MecanumVelocity(double v_x, double v_y, double omega);
};
```

**为什么需要这个结构体：**
- 麦轮的速度是 3 自由度的，与差速轮的 (v, omega) 不同
- `computeVelocityCommands` 返回的 `TwistStamped` 中 `linear.y` 对应麦轮横移
- `extractVelocity` 内部计算需要同时求解 v_x、v_y、omega 三个分量

#### 4.4.4 `TebConfig` — TEB 参数配置（麦轮版）

```cpp
struct TebConfig {
    // ===== 轨迹参数 =====
    double dt_ref = 0.3;               // 参考时间间隔 (s)
    double dt_hysteresis = 0.1;        // 时间间隔滞后带 (s)
    int min_samples = 3;               // 最少轨迹点数
    int max_samples = 500;             // 最多轨迹点数

    // ===== 障碍物参数 =====
    double min_obstacle_dist = 0.3;    // 最小障碍物距离 (m)
    double inflation_dist = 0.5;       // 障碍物膨胀距离 (m)

    // ===== 优化参数 =====
    int no_iterations = 5;             // 每次规划的优化迭代次数
    double weight_obstacle = 50.0;     // 障碍物代价权重
    double weight_optimaltime = 1.0;   // 时间最优权重
    double weight_shortest_path = 0.0; // 最短路径权重
    double weight_smoothness = 100.0;  // 平滑性权重
    double weight_kinematics = 500.0;  // 运动学约束权重

    // ===== 麦克纳姆轮速度参数 =====
    // 三个方向独立限速（与 MPC 的 max_v_ = 2.0 对齐）
    double max_vel_x = 0.5;            // 最大 x 方向速度 (m/s)
    double max_vel_y = 0.5;            // 最大 y 方向速度 (m/s) — 麦轮横移
    double max_vel_theta = 1.0;        // 最大角速度 (rad/s)

    // ===== 麦克纳姆轮加速度参数 =====
    // 三个方向独立加速度限制（与 MPC 的 max_a_v_ / max_a_w_ 对齐）
    double acc_lim_x = 2.0;            // 最大 x 方向加速度 (m/s^2)
    double acc_lim_y = 2.0;            // 最大 y 方向加速度 (m/s^2) — 麦轮横移
    double acc_lim_theta = 1.5;        // 最大角加速度 (rad/s^2)
};
```

**与差速轮版本的差异（用删除线标注被移除的字段）：**
- ~~`max_vel_x_backwards`~~：麦轮可任意方向平移，无正反之分
- ~~`min_turning_radius`~~：麦轮可原地旋转，转弯半径为 0
- ~~`is_holonomic`~~：麦轮版本恒为全向，无需开关
- **新增 `max_vel_y`**：麦轮横移速度独立限制
- **新增 `acc_lim_y`**：麦轮横移加速度独立限制

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
     * 麦轮版本：优化变量包含每个时间步的 (v_x, v_y, omega) 三自由度
     */
    void optimizeTEB();

    /**
     * @brief 从优化后的轨迹提取麦轮速度指令
     * @param robot_pose 当前机器人位姿
     * @param robot_vel 当前机器人速度（含 linear.x, linear.y, angular.z）
     * @return 麦轮速度指令 (v_x, v_y, omega)
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

    /**
     * @brief 麦轮运动学：从车体系速度推算世界坐标系位移
     * 与 MPC 包的 updateDiscreteModel 完全一致的运动学模型
     * @param v_x 车体系 x 方向速度
     * @param v_y 车体系 y 方向速度
     * @param omega 车体系角速度
     * @param theta 当前航向角
     * @param dt 时间步长
     * @return 世界坐标系下位移 [dx, dy, dtheta]
     */
    Eigen::Vector3d mecanumForwardKinematics(
        double v_x, double v_y, double omega,
        double theta, double dt) const;

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

    // TEB 配置（麦轮版）
    TebConfig config_;

    // 上一次速度指令（用于加速度约束，3自由度）
    MecanumVelocity last_velocity_;

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
| 新增 `MecanumVelocity` 结构体 | 麦轮 3 自由度速度需要独立封装，与差速轮 (v, omega) 区分 |
| 移除 `max_vel_x_backwards` | 麦轮可任意方向平移，正反方向速度限制相同 |
| 移除 `min_turning_radius` | 麦轮可原地旋转，无最小转弯半径约束 |
| 移除 `is_holonomic` 开关 | 麦轮版本恒为全向，不需要差速/全向切换 |
| 新增 `max_vel_y` / `acc_lim_y` | 麦轮横移方向有独立的速度/加速度限制 |
| 新增 `mecanumForwardKinematics()` | 麦轮运动学核心，与 MPC 的 B 矩阵一致，封装为方法避免重复 |
| `last_velocity_` 改为 `MecanumVelocity` | 3 自由度加速度约束需要记录上一步的 v_x、v_y、omega |
| 可视化用 `MarkerArray` | TEB 轨迹需要显示位姿点 + 速度方向箭头（含横移分量） |

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
    // 用户填写：归一化到 [-pi, pi]
    return 0.0;
}
```

#### 5.3.2 `TimedPose` / `MecanumVelocity` 构造
```cpp
TimedPose::TimedPose(const PoseSE2 &pose, double dt)
    : pose(pose), dt(dt) {}

MecanumVelocity::MecanumVelocity(double v_x, double v_y, double omega)
    : v_x(v_x), v_y(v_y), omega(omega) {}
```

#### 5.3.3 `mecanumForwardKinematics()`
与 MPC 包 `updateDiscreteModel` 的 B 矩阵完全一致的运动学：
```cpp
Eigen::Vector3d MyTebController::mecanumForwardKinematics(
    double v_x, double v_y, double omega,
    double theta, double dt) const {

    // 与 MPC mpc_controller.cpp:298 的 B 矩阵一致：
    // dx  = (v_x * cos(theta) - v_y * sin(theta)) * dt
    // dy  = (v_x * sin(theta) + v_y * cos(theta)) * dt
    // dth = omega * dt
    double
        dx  = (v_x * cos(theta) - v_y * sin(theta)) * dt,
        dy  = (v_x * sin(theta) + v_y * cos(theta)) * dt,
        dth = omega * dt;

    return Eigen::Vector3d(dx, dy, dth);
}
```

#### 5.3.4 `configure()`
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
    // ... 其余参数同理 ...

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

**参数清单（麦轮版，每个参数对应一对 declare + get）：**

| 参数名 | 默认值 | 对应 config_ 成员 | 说明 |
|--------|--------|-------------------|------|
| `dt_ref` | 0.3 | dt_ref | 参考时间间隔 |
| `dt_hysteresis` | 0.1 | dt_hysteresis | 时间间隔滞后带 |
| `min_samples` | 3 | min_samples | 最少轨迹点数 |
| `max_samples` | 500 | max_samples | 最多轨迹点数 |
| `min_obstacle_dist` | 0.3 | min_obstacle_dist | 最小障碍物距离 |
| `inflation_dist` | 0.5 | inflation_dist | 障碍物膨胀距离 |
| `no_iterations` | 5 | no_iterations | 优化迭代次数 |
| `weight_obstacle` | 50.0 | weight_obstacle | 障碍物代价权重 |
| `weight_optimaltime` | 1.0 | weight_optimaltime | 时间最优权重 |
| `weight_smoothness` | 100.0 | weight_smoothness | 平滑性权重 |
| `weight_kinematics` | 500.0 | weight_kinematics | 运动学约束权重 |
| **`max_vel_x`** | **0.5** | max_vel_x | **最大 x 方向速度** |
| **`max_vel_y`** | **0.5** | max_vel_y | **最大 y 方向速度（横移）** |
| **`max_vel_theta`** | **1.0** | max_vel_theta | **最大角速度** |
| **`acc_lim_x`** | **2.0** | acc_lim_x | **最大 x 方向加速度** |
| **`acc_lim_y`** | **2.0** | acc_lim_y | **最大 y 方向加速度（横移）** |
| **`acc_lim_theta`** | **1.5** | acc_lim_theta | **最大角加速度** |
| `transform_tolerance` | 0.1 | transform_tolerance_ | TF 容差 |

#### 5.3.5 `setPlan()`
```cpp
void MyTebController::setPlan(const nav_msgs::msg::Path &path) {
    global_plan_ = path;
}
```

#### 5.3.6 `computeVelocityCommands()`
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

    // 5. 提取麦轮速度指令（3自由度）
    //    调用 extractVelocity()

    // 6. 发布可视化
    //    调用 publishVisualization()

    // 7. 封装 TwistStamped（麦轮3自由度）
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = "base_link";
    // cmd_vel.twist.linear.x = ...   // 麦轮前进/后退
    // cmd_vel.twist.linear.y = ...   // 麦轮横移（区别于差速轮！）
    // cmd_vel.twist.angular.z = ...  // 角速度

    return cmd_vel;
}
```

#### 5.3.7 TEB 算法方法（空方法体）
```cpp
void MyTebController::initializeTrajectory(
    const nav_msgs::msg::Path &global_path,
    const PoseSE2 &start_pose) {
    // 用户填写：从全局路径采样生成初始 teb_trajectory_
}

void MyTebController::optimizeTEB() {
    // 用户填写：构建优化问题并求解
    // 麦轮版：优化变量为每个时间步的 (v_x, v_y, omega)
    // 可使用 mecanumForwardKinematics() 推算轨迹点
}

geometry_msgs::msg::TwistStamped MyTebController::extractVelocity(
    const PoseSE2 &robot_pose,
    const geometry_msgs::msg::Twist &robot_vel) {
    // 用户填写：从 teb_trajectory_ 的前两个点计算麦轮速度指令
    // 需同时求解 v_x, v_y, omega 三个分量
    geometry_msgs::msg::TwistStamped cmd_vel;
    return cmd_vel;
}

bool MyTebController::checkTrajectoryFeasibility() {
    // 用户填写：检查碰撞和运动学约束
    // 麦轮版：检查 v_x/v_y/omega 是否超限，无需检查转弯半径
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
    // 建议：用箭头 Marker 显示每个轨迹点的 v_x/v_y 合成方向
}
```

#### 5.3.8 生命周期方法
```cpp
void MyTebController::activate() { RCLCPP_INFO(logger_, "插件已激活"); }
void MyTebController::deactivate() { RCLCPP_INFO(logger_, "插件已停用"); }
void MyTebController::cleanup() { RCLCPP_INFO(logger_, "插件已清理"); }
```

#### 5.3.9 `setSpeedLimit()`
```cpp
void MyTebController::setSpeedLimit(const double &speed_limit, const bool &percentage) {
    (void)speed_limit;
    (void)percentage;
    RCLCPP_INFO(logger_, "收到限速指令，当前插件尚未实现具体的限速逻辑");
}
```

#### 5.3.10 插件注册（文件最后一行）
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
```cpp
nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".param_name", rclcpp::ParameterValue(default_value));
node->get_parameter(plugin_name_ + ".param_name", member_variable);
```

### 6.5 日志模式
```cpp
RCLCPP_INFO(logger_, "自定义TEB控制器配置完成");
```

---

## 7. Nav2 配置集成

### 7.1 在 `nav2_params_nav_diy.yaml` 中添加 TEB 配置（麦轮版）

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    # 切换到 TEB 控制器时使用：
    # controller_plugins: ["FollowPath"]
    # FollowPath:
    #   plugin: "my_teb_controller/MyTebController"
    #   # 轨迹参数
    #   dt_ref: 0.3
    #   dt_hysteresis: 0.1
    #   min_samples: 3
    #   max_samples: 500
    #   # 障碍物参数
    #   min_obstacle_dist: 0.3
    #   inflation_dist: 0.5
    #   # 优化参数
    #   no_iterations: 5
    #   weight_obstacle: 50.0
    #   weight_optimaltime: 1.0
    #   weight_smoothness: 100.0
    #   weight_kinematics: 500.0
    #   # 麦轮速度参数（3自由度）
    #   max_vel_x: 0.5
    #   max_vel_y: 0.5        # 麦轮横移速度
    #   max_vel_theta: 1.0
    #   # 麦轮加速度参数（3自由度）
    #   acc_lim_x: 2.0
    #   acc_lim_y: 2.0        # 麦轮横移加速度
    #   acc_lim_theta: 1.5
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
- 若横移不生效：确认 `cmd_vel.twist.linear.y` 被正确赋值（麦轮特有）

---

## 9. 任务执行顺序

1. **Step 1**：新建 `plugins.xml`
2. **Step 2**：重写 `package.xml`（补全依赖）
3. **Step 3**：重写 `CMakeLists.txt`（补全构建逻辑）
4. **Step 4**：重写 `teb_controller.hpp`（补全数据结构 + 成员 + 方法声明）
5. **Step 5**：重写 `teb_controller.cpp`（补全方法骨架 + 插件注册）
6. **Step 6**：`colcon build --packages-select my_teb_controller` 验证编译通过
