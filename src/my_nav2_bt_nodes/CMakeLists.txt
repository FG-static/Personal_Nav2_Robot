cmake_minimum_required(VERSION 3.8)
project(my_nav2_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_library(my_manage_trajectory_bt_node SHARED
    src/manage_trajectory_action.cpp
)

target_compile_definitions(my_manage_trajectory_bt_node PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(my_manage_trajectory_bt_node
    behaviortree_cpp
    geometry_msgs
    nav2_behavior_tree
    nav_msgs
)

install(TARGETS my_manage_trajectory_bt_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(my_manage_trajectory_bt_node)
ament_export_dependencies(
    behaviortree_cpp
    geometry_msgs
    nav2_behavior_tree
    nav_msgs
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
