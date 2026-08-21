cmake_minimum_required(VERSION 3.10)
project(rm_interfaces)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
## add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

###########
## Build ##
###########

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target.msg"
  "msg/Gimbal.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

#############
## Install ##
#############

ament_package()