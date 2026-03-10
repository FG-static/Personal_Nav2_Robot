cmake_minimum_required(VERSION 3.8)
project(nav2_visual_processing)

# Modern C++ settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Suppress policy warnings coming from dependencies (e.g., PCL/FLANN)
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

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
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pluginlib REQUIRED)
find_package(small_gicp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Executable
add_executable(visual_processor_node src/visual_processor.cpp)

ament_target_dependencies(visual_processor_node
  rclcpp
  sensor_msgs
  message_filters
  cv_bridge
  Eigen3
  OpenCV
  PCL
  pcl_conversions
  pluginlib
  small_gicp
  geometry_msgs 
  tf2_ros
)

# Link libraries
target_link_libraries(visual_processor_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

# Install executables
install(TARGETS visual_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
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
