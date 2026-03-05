cmake_minimum_required(VERSION 3.8)
project(nav2_visual_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Executable
add_executable(visual_processor_node src/visual_processor.cpp)

ament_target_dependencies(visual_processor_node
  rclcpp
  sensor_msgs
  message_filters
  cv_bridge
  Eigen3
  OpenCV
)

# Install executables
install(TARGETS visual_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files (if any)
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# Install URDF files (if any)
# install(DIRECTORY urdf
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
