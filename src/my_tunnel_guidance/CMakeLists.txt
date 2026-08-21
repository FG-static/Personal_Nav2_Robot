cmake_minimum_required(VERSION 3.8)
project(my_tunnel_guidance)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}_geometry
    src/tunnel_geometry_estimator.cpp
)
target_include_directories(${PROJECT_NAME}_geometry PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_geometry
    ${PCL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_geometry
    Eigen3
)

add_library(${PROJECT_NAME}_timing
    src/pointcloud_timing.cpp
)
target_include_directories(${PROJECT_NAME}_timing PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_timing
    rclcpp
    sensor_msgs
)

add_executable(tunnel_guidance_node
    src/tunnel_guidance_node.cpp
)
target_link_libraries(tunnel_guidance_node
    ${PROJECT_NAME}_geometry
    ${PROJECT_NAME}_timing
    ${PCL_LIBRARIES}
)
ament_target_dependencies(tunnel_guidance_node
    rclcpp
    rclcpp_action
    nav2_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    rm_interfaces
    visualization_msgs
    tf2
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    pcl_conversions
    Eigen3
)

install(TARGETS
    tunnel_guidance_node
    ${PROJECT_NAME}_geometry
    ${PROJECT_NAME}_timing
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)
install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_geometry ${PROJECT_NAME}_timing)
ament_export_dependencies(rclcpp rclcpp_action nav2_msgs sensor_msgs geometry_msgs nav_msgs std_msgs rm_interfaces visualization_msgs tf2 tf2_ros tf2_eigen tf2_geometry_msgs pcl_conversions Eigen3)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_tunnel_geometry_estimator
        test/test_tunnel_geometry_estimator.cpp
    )
    if(TARGET test_tunnel_geometry_estimator)
        target_link_libraries(test_tunnel_geometry_estimator ${PROJECT_NAME}_geometry)
        target_include_directories(test_tunnel_geometry_estimator PRIVATE include)
    endif()

    ament_add_gtest(test_pointcloud_timing
        test/test_pointcloud_timing.cpp
    )
    if(TARGET test_pointcloud_timing)
        target_link_libraries(test_pointcloud_timing ${PROJECT_NAME}_timing)
        target_include_directories(test_pointcloud_timing PRIVATE include)
        ament_target_dependencies(test_pointcloud_timing sensor_msgs)
    endif()
endif()

ament_package()
