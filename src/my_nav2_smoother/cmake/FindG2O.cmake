set(_g2o_hints
  /opt/ros/$ENV{ROS_DISTRO}/lib/${CMAKE_LIBRARY_ARCHITECTURE}
  /opt/ros/$ENV{ROS_DISTRO}/lib
  /opt/ros/$ENV{ROS_DISTRO}/include
)

find_path(G2O_INCLUDE_DIR
  NAMES g2o/core/sparse_optimizer.h
  HINTS ${_g2o_hints}
)

find_library(G2O_CORE_LIBRARY
  NAMES g2o_core
  HINTS ${_g2o_hints}
)

find_library(G2O_STUFF_LIBRARY
  NAMES g2o_stuff
  HINTS ${_g2o_hints}
)

find_library(G2O_SOLVER_EIGEN_LIBRARY
  NAMES g2o_solver_eigen
  HINTS ${_g2o_hints}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(G2O
  REQUIRED_VARS
    G2O_INCLUDE_DIR
    G2O_CORE_LIBRARY
    G2O_STUFF_LIBRARY
    G2O_SOLVER_EIGEN_LIBRARY
)

if(G2O_FOUND AND NOT TARGET G2O::all)
  add_library(G2O::all INTERFACE IMPORTED)
  set_target_properties(G2O::all PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${G2O_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES
      "${G2O_CORE_LIBRARY};${G2O_STUFF_LIBRARY};${G2O_SOLVER_EIGEN_LIBRARY}"
  )
endif()

set(G2O_INCLUDE_DIRS "${G2O_INCLUDE_DIR}")
set(G2O_LIBRARIES
  "${G2O_CORE_LIBRARY};${G2O_STUFF_LIBRARY};${G2O_SOLVER_EIGEN_LIBRARY}"
)
