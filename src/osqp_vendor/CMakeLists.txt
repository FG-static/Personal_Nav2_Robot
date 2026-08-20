cmake_minimum_required(VERSION 3.9)

project(osqp_vendor)

find_package(ros_environment REQUIRED)

if($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED)
elseif($ENV{ROS_VERSION} EQUAL 2)
  find_package(ament_cmake REQUIRED)
endif()

macro(build_osqp)
  set(git_tag "v0.6.2")
  set(extra_cmake_args)

  get_property(multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
  if(NOT multi_config AND DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()

  # Local OSQP 0.6.2 + QDLDL (curl'd into third_party/osqp). Avoids git/apt.
  get_filename_component(_osqp_src
    "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/osqp" ABSOLUTE)
  if(NOT EXISTS "${_osqp_src}/CMakeLists.txt")
    message(FATAL_ERROR "OSQP sources not found at ${_osqp_src}")
  endif()

  include(ExternalProject)
  externalproject_add(osqp-${git_tag}
    SOURCE_DIR "${_osqp_src}"
    UPDATE_COMMAND ""
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/osqp_install
      -DUNITTESTS=OFF
      -DENABLE_MKL_PARDISO=OFF
      ${extra_cmake_args}
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/osqp_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
    USE_SOURCE_PERMISSIONS
    PATTERN config.h EXCLUDE
  )
endmacro()

build_osqp()

if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    LIBRARIES osqp
    DEPENDS osqp
    CFG_EXTRAS osqp_vendor-extras.cmake)
elseif($ENV{ROS_VERSION} EQUAL 2)
  ament_export_libraries(osqp)
  ament_export_dependencies(osqp)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package(CONFIG_EXTRAS cmake/osqp_vendor-extras.cmake)
endif()
