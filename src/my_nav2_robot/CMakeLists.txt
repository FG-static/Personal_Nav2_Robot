cmake_minimum_required(VERSION 3.8)
project(my_nav2_robot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(gz-sim8 REQUIRED
  HINTS /opt/ros/$ENV{ROS_DISTRO}/opt/gz_sim_vendor/lib/cmake)
find_package(gz-plugin2 REQUIRED
  HINTS /opt/ros/$ENV{ROS_DISTRO}/opt/gz_plugin_vendor/lib/cmake)

add_executable(robot_driver src/robot_driver.cpp)
add_executable(odom_evaluator src/odom_evaluator.cpp)
add_executable(simulated_imu_publisher src/simulated_imu_publisher.cpp)
add_library(my_nav2_robot_mid360_gz_plugin SHARED src/mid360_gz_plugin.cpp)

ament_target_dependencies(robot_driver
    rclcpp
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
)

ament_target_dependencies(odom_evaluator
    rclcpp
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_eigen
    Sophus
)

target_include_directories(simulated_imu_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(simulated_imu_publisher
    rclcpp
    geometry_msgs
    sensor_msgs
    nav_msgs
)

target_include_directories(my_nav2_robot_mid360_gz_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(my_nav2_robot_mid360_gz_plugin
    rclcpp
    sensor_msgs
)

target_link_libraries(my_nav2_robot_mid360_gz_plugin
  gz-sim8::core
  gz-plugin2::register
)

# 安装
install(TARGETS
  robot_driver
  odom_evaluator
  simulated_imu_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS my_nav2_robot_mid360_gz_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
