cmake_minimum_required(VERSION 3.8)
project(my_planning_metrics)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)

install(DIRECTORY include/
    DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(nav2_costmap_2d nav_msgs)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(path_metrics_test
        test/path_metrics_test.cpp
    )
    if(TARGET path_metrics_test)
        target_include_directories(path_metrics_test PRIVATE include)
        ament_target_dependencies(path_metrics_test nav2_costmap_2d nav_msgs)
    endif()
endif()

ament_package()
